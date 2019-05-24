// Copyright 2019 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ros/console.h>
#include <ros_utilities/scoped_timer.h>
#include <trajectory_math/cubic_spiral.h>
#include <trajectory_math/curve_algorithms.h>
#include <trajectory_math/path_algorithms.h>
#include <trajectory_math/point_algorithms.h>
#include <trajectory_math/trajectory.h>
#include <velocity_planner/velocity_planner_introspection.h>
#include <velocity_planner/velocity_profile_search.h>
#include <velocity_planner/velocity_search_visited_grid.h>

#include <Eigen/Geometry>
#include <deque>
#include <queue>
#include <set>

namespace bookbot {

namespace {

enum class VelocitySearchLatticePrimitiveType {
  kConstantAcceleration,
  kTurnInPlace,
  kStandStill
};

struct VelocitySearchNode {
  VelocitySearchNode(
      int time_index_in, int velocity_index_in, double distance_traveled_in,
      double yaw_in, double x_position_in, double y_position_in,
      double curvature_in, double previous_acceleration_in,
      VelocitySearchLatticePrimitiveType previous_primitive_type_in,
      double cost_to_come_in, double cost_to_go_in, int parent_index_in)
      : time_index(time_index_in),
        velocity_index(velocity_index_in),
        distance_traveled(distance_traveled_in),
        yaw(yaw_in),
        x_position(x_position_in),
        y_position(y_position_in),
        curvature(curvature_in),
        previous_acceleration(previous_acceleration_in),
        previous_primitive_type(previous_primitive_type_in),
        cost_to_come(cost_to_come_in),
        cost_to_go(cost_to_go_in),
        parent_index(parent_index_in) {}

  Eigen::Vector2d Position() const { return {x_position, y_position}; }

  // Search state
  int time_index;
  int velocity_index;
  double distance_traveled;
  double yaw;

  // Cached path information
  double x_position;
  double y_position;
  double curvature;

  // Acceleration used in previous primitive and previous primitive type
  double previous_acceleration;
  VelocitySearchLatticePrimitiveType previous_primitive_type;

  // Search node values
  double cost_to_come;
  double cost_to_go;
  int parent_index;
};

std::ostream& operator<<(std::ostream& stream,
                         const VelocitySearchLatticePrimitiveType& type);

std::ostream& operator<<(std::ostream& stream, const VelocitySearchNode& node);

std::vector<VelocitySearchNode> EnumerateNeighbors(
    const VelocitySearchNode& current_node, const VelocityPlannerParams& params,
    const VelocitySearchVisitedGrid& visited_set, double velocity_resolution,
    int zero_velocity_index, double time_resolution, const Path& desired_path,
    int current_node_index);

}  // namespace

Trajectory PerformVelocityProfileSearch(
    const Path& desired_path, const Trajectory1DPoint& initial_state,
    const VelocityPlannerParams& params, const Trajectory& previous_trajectory,
    const SignedDistanceField& distance_field) {
  // v1   v2  v3  v4
  //  |   |   |   |
  // [0 1 2 3 4 5 6]  velocity_resolution = (max - min) / (vel_bins - 1)
  //   \_/ \_/ \_/                        = (6 - 0) / (4 - 1) = 2
  // velocity_resolution
  const double velocity_resolution =
      (params.velocity_bounds.constraint_interval.max_value -
       params.velocity_bounds.constraint_interval.min_value) /
      static_cast<double>(params.velocity_bins - 1);

  const double time_resolution =
      params.planning_time_horizon / static_cast<double>(params.time_bins);

  auto min_time_to_end = [&desired_path,
                          &params](const VelocitySearchNode& node) -> double {
    const double distance =
        desired_path.back().distance_along_path - node.distance_traveled;
    return distance / params.velocity_bounds.constraint_interval.max_value;
  };

  auto search_node_comparator = [](const VelocitySearchNode& a,
                                   const VelocitySearchNode& b) -> bool {
    return (a.cost_to_come + a.cost_to_go) > (b.cost_to_come + b.cost_to_go);
  };

  auto bin_velocity = [&params, velocity_resolution](double velocity) -> int {
    return std::round(
        (velocity - params.velocity_bounds.constraint_interval.min_value) /
        velocity_resolution);
  };

  const int zero_velocity_index = bin_velocity(0);

  auto visited_set =
      VelocitySearchVisitedGrid(params.visited_grid_distance_resolution);
  std::vector<VelocitySearchNode> closed_list;
  std::deque<VelocitySearchNode> velocity_profile;
  std::priority_queue<VelocitySearchNode, std::vector<VelocitySearchNode>,
                      decltype(search_node_comparator)>
      open_list(search_node_comparator);

  PathPoint initial_path_point = InterpolateCubicSpiralPathByDistanceAlongPath(
      initial_state.distance, std::begin(desired_path), std::end(desired_path));
  int initial_v_index = bin_velocity(initial_state.velocity);
  auto initial_node = VelocitySearchNode(
      0, initial_v_index, initial_state.distance, initial_path_point.yaw,
      initial_path_point.x, initial_path_point.y, initial_path_point.curvature,
      initial_state.acceleration,
      VelocitySearchLatticePrimitiveType::kConstantAcceleration, 0, 0, -1);
  initial_node.cost_to_go =
      min_time_to_end(initial_node) * params.travel_time_weight;
  open_list.push(initial_node);
  while (!open_list.empty()) {
    ScopedTimer timer("VelocitySearchTreeNodeEvaluation");
    VelocitySearchNode current_node = open_list.top();
    open_list.pop();

    // Check and insert into visited set
    if (!visited_set.Insert(current_node.time_index,
                            current_node.velocity_index,
                            current_node.distance_traveled)) {
      // Insert returns false if node already in visited set
      continue;
    }
    closed_list.push_back(current_node);

    // Check for goal
    double current_min_time_to_end = min_time_to_end(current_node);
    if ((current_min_time_to_end < 0.5 * time_resolution &&
         current_node.velocity_index == zero_velocity_index) ||
        current_node.time_index >= params.time_bins) {
      // At goal or horizon -> return best velocity_profile
      int parent_index = current_node.parent_index;
      velocity_profile.push_back(current_node);
      while (parent_index != -1) {
        VelocitySearchNode& out_node = closed_list[parent_index];
        parent_index = out_node.parent_index;
        velocity_profile.push_front(out_node);
      }
      break;
    }

    // Generate neighbor candidate set (no costs are computed yet)
    std::vector<VelocitySearchNode> candidate_neighbor_nodes =
        EnumerateNeighbors(current_node, params, visited_set,
                           velocity_resolution, zero_velocity_index,
                           time_resolution, desired_path,
                           closed_list.size() - 1);

    // Evaluate neighbors and add them to the open list
    for (VelocitySearchNode& neighbor_node : candidate_neighbor_nodes) {
      const double neighbor_velocity =
          params.velocity_bounds.constraint_interval.min_value +
          static_cast<double>(neighbor_node.velocity_index) *
              velocity_resolution;

      neighbor_node.cost_to_go =
          min_time_to_end(neighbor_node) * params.travel_time_weight;

      double obstacle_cost = 0;
      double dist_to_obstacles = 0;

      // Check constraints
      // Lateral acceleration constraints
      const double lateral_acceleration =
          neighbor_velocity * neighbor_velocity * neighbor_node.curvature;
      if (!params.lateral_acceleration_bounds.IsSatisfied(
              lateral_acceleration)) {
        continue;
      }

      // Max yaw change
      if (AngleDifference(neighbor_node.yaw, current_node.yaw) >
              params.max_yaw_change_for_nonzero_velocity &&
          neighbor_node.velocity_index != zero_velocity_index) {
        continue;
      }

      // Obstacle constraints
      // The occupancy_grid is already bloated by half the robot width so we
      // want to check two points to get a two circle coverage of the robot
      // shape.
      //   __________________
      //  |                  |
      //  |   _|_ ____ _|_   |
      //  |    | offset |    |
      //  |__________________|
      //       ^       ^
      //   base_link   base_link + offset
      //
      const Eigen::Vector2d neighbor_position = neighbor_node.Position();
      const Eigen::Vector2d neighbor_front_position =
          neighbor_position +
          Eigen::Rotation2D<double>(neighbor_node.yaw) *
              Eigen::Vector2d(params.obstacle_check_front_offset, 0);
      if (!params.override_obstacles) {
        double distance_to_obstacles =
            distance_field.Evaluate(neighbor_position);
        if (distance_to_obstacles <= params.robot_radius) {
          continue;
        }
        double front_distance_to_obstacles =
            distance_field.Evaluate(neighbor_front_position);
        if (front_distance_to_obstacles <= params.robot_radius) {
          continue;
        }
      }

      // Compute costs
      const double longitudinal_acceleration_cost =
          neighbor_node.previous_acceleration *
          neighbor_node.previous_acceleration *
          params.longitudinal_acceleration_weight;
      const double lateral_acceleration_cost =
          lateral_acceleration * lateral_acceleration *
          params.lateral_acceleration_weight;
      const double elapsed_time_cost = time_resolution * time_resolution;
      const double desired_velocity_deviation =
          params.desired_velocity_bounds.constraint_interval.max_value -
          neighbor_velocity;
      const double desired_velocity_deviation_cost =
          desired_velocity_deviation * desired_velocity_deviation *
          params.desired_velocity_weight;
      double incremental_node_cost =
          longitudinal_acceleration_cost + lateral_acceleration_cost +
          elapsed_time_cost + desired_velocity_deviation_cost;
      neighbor_node.cost_to_come =
          current_node.cost_to_come + incremental_node_cost;
      open_list.push(neighbor_node);
    }
  }

  auto search_node_to_trajectory_point =
      [&initial_state, &time_resolution, &velocity_resolution,
       &params](const VelocitySearchNode& node) -> TrajectoryPoint {
    TrajectoryPoint point;
    point.time = initial_state.time + node.time_index * time_resolution;
    point.x = node.x_position;
    point.y = node.y_position;
    point.distance_along_path = node.distance_traveled;
    point.yaw = node.yaw;
    point.curvature = node.curvature;
    point.velocity =
        params.velocity_bounds.constraint_interval.min_value +
        static_cast<double>(node.velocity_index) * velocity_resolution;
    return point;
  };

  // Visualize expanded Tree
  std::vector<std::pair<TrajectoryPoint, TrajectoryPoint>> expanded_tree;
  for (const auto& node : closed_list) {
    if (node.parent_index == -1) {
      continue;
    }
    expanded_tree.emplace_back(
        search_node_to_trajectory_point(closed_list[node.parent_index]),
        search_node_to_trajectory_point(node));
  }
  PublishIntrospection("expanded_velocity_tree", "odom", expanded_tree);

  // Convert optimal node sequence to trajectory
  Trajectory trajectory;
  for (const auto& node : velocity_profile) {
    trajectory.push_back(search_node_to_trajectory_point(node));
  }
  return trajectory;
}

namespace {

std::ostream& operator<<(std::ostream& stream,
                         const VelocitySearchLatticePrimitiveType& type) {
  switch (type) {
    case VelocitySearchLatticePrimitiveType::kConstantAcceleration:
      return stream << "<ConstantAccel>";
    case VelocitySearchLatticePrimitiveType::kTurnInPlace:
      return stream << "<TurnInPlace>";
    case VelocitySearchLatticePrimitiveType::kStandStill:
      return stream << "<StandStill>";
  }
  return stream;
}

std::ostream& operator<<(std::ostream& stream, const VelocitySearchNode& node) {
  stream << "[t:" << node.time_index << " v:" << node.velocity_index
         << " d:" << node.distance_traveled << " yaw:" << node.yaw
         << " type:" << node.previous_primitive_type
         << " c2c:" << node.cost_to_come << " ctg:" << node.cost_to_go << "]";
  return stream;
}

std::vector<VelocitySearchNode> EnumerateNeighbors(
    const VelocitySearchNode& current_node, const VelocityPlannerParams& params,
    const VelocitySearchVisitedGrid& visited_set, double velocity_resolution,
    int zero_velocity_index, double time_resolution, const Path& desired_path,
    int current_node_index) {
  std::vector<VelocitySearchNode> candidate_neighbor_nodes;
  // Add constant acceleration_options
  for (int neighbor_velocity_index = 0;
       neighbor_velocity_index < params.velocity_bins;
       ++neighbor_velocity_index) {
    const double acceleration =
        static_cast<double>(neighbor_velocity_index -
                            current_node.velocity_index) *
        velocity_resolution / time_resolution;
    if (!params.acceleration_bounds.IsSatisfied(acceleration)) {
      continue;
    }

    const double current_velocity =
        params.velocity_bounds.constraint_interval.min_value +
        static_cast<double>(current_node.velocity_index) * velocity_resolution;
    const double neighbor_distance_traveled =
        current_node.distance_traveled + current_velocity * time_resolution +
        0.5 * acceleration * time_resolution * time_resolution;

    // Check if its past the end of the path
    if (neighbor_distance_traveled > desired_path.back().distance_along_path -
                                         params.path_end_buffer_distance) {
      continue;
    }

    // Check visited
    if (visited_set.Contains(current_node.time_index + 1,
                             neighbor_velocity_index,
                             neighbor_distance_traveled)) {
      continue;
    }

    auto neighbor_path_point = InterpolateCubicSpiralPathByDistanceAlongPath(
        neighbor_distance_traveled, std::begin(desired_path),
        std::end(desired_path));

    candidate_neighbor_nodes.emplace_back(
        current_node.time_index + 1, neighbor_velocity_index,
        neighbor_distance_traveled, neighbor_path_point.yaw,
        neighbor_path_point.x, neighbor_path_point.y,
        neighbor_path_point.curvature, acceleration,
        VelocitySearchLatticePrimitiveType::kConstantAcceleration, 0, 0,
        current_node_index);
  }

  // Define turn in place options (equivalent to turning M_PI/2 in 1 second)
  const std::array<double, 2> turn_in_place_yaw_change_options = {
      -M_PI_2 * time_resolution, M_PI_2 * time_resolution};

  if (current_node.velocity_index == zero_velocity_index) {
    // Add turn-in-place options
    for (double yaw_change : turn_in_place_yaw_change_options) {
      // Skip visited list check for turn-in-place options
      candidate_neighbor_nodes.emplace_back(
          current_node.time_index + 1, zero_velocity_index,
          current_node.distance_traveled, current_node.yaw + yaw_change,
          current_node.x_position, current_node.y_position,
          current_node.curvature, 0,
          VelocitySearchLatticePrimitiveType::kTurnInPlace, 0, 0,
          current_node_index);
    }

    // Add standstill option
    // Skip visited list check for standstill options
    candidate_neighbor_nodes.emplace_back(
        current_node.time_index + 1, zero_velocity_index,
        current_node.distance_traveled, current_node.yaw,
        current_node.x_position, current_node.y_position,
        current_node.curvature, 0,
        VelocitySearchLatticePrimitiveType::kStandStill, 0, 0,
        current_node_index);
  }

  return candidate_neighbor_nodes;
}

}  // namespace

}  // namespace bookbot

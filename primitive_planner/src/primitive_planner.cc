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

#include <primitive_planner/primitive.h>
#include <primitive_planner/primitive_planner.h>
#include <primitive_planner/primitive_planner_introspection.h>
#include <primitive_planner/primitive_search_node.h>
#include <primitive_planner/visited_grid.h>
#include <ros/console.h>
#include <ros_utilities/scoped_timer.h>
#include <trajectory_math/curve_algorithms.h>
#include <trajectory_math/path_algorithms.h>
#include <trajectory_math/point_algorithms.h>

#include <queue>

namespace bookbot {

std::vector<PathPrimitiveWithEvaluation> GeneratePrimitiveSet(
    double primitive_length, double primitive_min_turn_radius,
    int primitive_angle_samples) {
  std::vector<PathPrimitiveWithEvaluation> primitive_set;

  // StandStill
  primitive_set.push_back(GenerateStandStillPrimitive());

  // Straight
  primitive_set.push_back(GenerateCubicSpiralPrimitive(primitive_length, 0.));

  // Get delta yaw if turning at min turn radius: delta_yaw = curvature * length
  double max_delta_yaw = 1 / primitive_min_turn_radius * primitive_length;
  double delta_yaw_spacing =
      max_delta_yaw / static_cast<double>(primitive_angle_samples);

  // Left turn primitives
  for (double delta_yaw = delta_yaw_spacing; delta_yaw < max_delta_yaw;
       delta_yaw += delta_yaw_spacing) {
    primitive_set.push_back(
        GenerateCubicSpiralPrimitive(primitive_length, delta_yaw));
  }

  // Turn-in-place left
  primitive_set.push_back(GenerateTurnInPlacePrimitive(max_delta_yaw));

  // Right turn primitives
  for (double delta_yaw = -delta_yaw_spacing; delta_yaw > -max_delta_yaw;
       delta_yaw -= delta_yaw_spacing) {
    primitive_set.push_back(
        GenerateCubicSpiralPrimitive(primitive_length, delta_yaw));
  }

  // Turn-in-place right
  primitive_set.push_back(GenerateTurnInPlacePrimitive(-max_delta_yaw));

  return primitive_set;
}

Path PlanPrimitivePath(const Path& clicked_path, const PathPoint& start_point,
                       const PrimitivePlanningParams& params,
                       const std::vector<PathPrimitiveWithEvaluation>&
                           primitive_set_with_evaluation,
                       const SignedDistanceField& distance_field,
                       const SignedDistanceField& clicked_path_field) {
  ScopedTimer timer("PlanPrimitivePath");

  // Extract set of precomputed primitives
  std::vector<PathPrimitive> primitive_set;
  for (const auto& primitive_with_evaluation : primitive_set_with_evaluation) {
    primitive_set.push_back(primitive_with_evaluation.GetPrimitive());
  }

  // Define helper functions
  auto squared_dist_to_goal =
      [&clicked_path](const PathPrimitiveSearchNode& node) -> double {
    return Eigen::Vector2d(clicked_path.back().Position() - node.Position())
        .squaredNorm();
  };

  auto cost_to_goal = [&clicked_path,
                       &params](const PathPrimitiveSearchNode& node) -> double {
    auto matched_point = MatchToPath(node.Position(), std::begin(clicked_path),
                                     std::end(clicked_path));
    auto distance = clicked_path.back().distance_along_path -
                    matched_point.distance_along_path;
    auto squared_offset =
        Eigen::Vector2d(matched_point.Position() - node.Position())
            .squaredNorm();
    auto angle_offset = AngleDifference(node.yaw, matched_point.yaw);
    auto min_steps_to_goal = distance / params.primitive_length;
    return min_steps_to_goal * params.travel_cost_weight +
           squared_offset * params.path_end_divergence_weight +
           angle_offset * angle_offset * params.path_end_alignment_weight;
  };

  auto primitive_node_comparator =
      [](const PathPrimitiveSearchNode& a,
         const PathPrimitiveSearchNode& b) -> bool {
    return (a.cost_from_start + a.cost_to_goal) >
           (b.cost_from_start + b.cost_to_goal);
  };

  // Initialize data structures for A* search
  auto visited_set = VisitedGrid(params.visited_grid_spatial_resolution,
                                 params.num_visited_grid_angle_bins);
  std::vector<PathPrimitiveSearchNode> closed_list;
  std::deque<PathPrimitiveSearchNode> primitive_path;
  std::priority_queue<PathPrimitiveSearchNode,
                      std::vector<PathPrimitiveSearchNode>,
                      decltype(primitive_node_comparator)>
      open_list(primitive_node_comparator);

  // Perform search
  auto initial_node =
      PathPrimitiveSearchNode(start_point.x, start_point.y, start_point.yaw, 0,
                              0, PathPrimitive(), 0, 0, -1);
  initial_node.cost_to_goal = cost_to_goal(initial_node);
  open_list.push(initial_node);
  while (!open_list.empty()) {
    ScopedTimer timer("PrimitiveTreeNodeEvaluation");
    PathPrimitiveSearchNode current_node = open_list.top();
    open_list.pop();

    // Check and insert into visited set
    if (!visited_set.Insert(current_node.x, current_node.y, current_node.yaw) &&
        current_node.previous_primitive.primitive_type !=
            PathPrimitiveType::kStandStill) {
      // Insert returns false if node already in visited set
      continue;
    }
    closed_list.push_back(current_node);

    // Check for goal
    double current_squared_dist_to_goal = squared_dist_to_goal(current_node);
    if (current_squared_dist_to_goal <
            2 * params.visited_grid_spatial_resolution ||
        current_node.tree_depth >= params.horizon) {
      // At goal or horizon -> return best path
      int parent_index = current_node.parent_index;
      primitive_path.push_back(current_node);
      while (parent_index != -1) {
        auto& out_node = closed_list[parent_index];
        parent_index = out_node.parent_index;
        primitive_path.push_front(out_node);
      }
      break;
    }

    // Add neighbors using primitive set
    for (const PathPrimitive& primitive : primitive_set) {
      auto neighbor_state = ApplyPathPrimitive(primitive, current_node.State());
      auto neighbor_node = PathPrimitiveSearchNode(
          neighbor_state.x, neighbor_state.y, neighbor_state.yaw,
          neighbor_state.distance_along_path, current_node.tree_depth + 1,
          primitive, 0, 0, closed_list.size() - 1);
      if (primitive.primitive_type != PathPrimitiveType::kStandStill &&
          visited_set.Contains(neighbor_node.x, neighbor_node.y,
                               neighbor_node.yaw)) {
        continue;
      }
      neighbor_node.cost_to_goal = cost_to_goal(neighbor_node);

      // Obstacle constraints
      // Check two points to get a two circle coverage of the robot shape.
      //   __________________
      //  |                  |
      //  |   _|_ ____ _|_   |
      //  |    | offset |    |
      //  |__________________|
      //       ^        ^
      //   base_link   base_link + offset
      //
      //
      double obstacle_cost = 0;
      double dist_to_obstacles = 0;
      const Eigen::Vector2d neighbor_position = neighbor_node.Position();
      const Eigen::Vector2d neighbor_front_position =
          neighbor_position +
          Eigen::Rotation2D<double>(neighbor_node.yaw) *
              Eigen::Vector2d(params.obstacle_check_front_offset, 0);
      if (params.require_perception) {
        dist_to_obstacles =
            distance_field.Evaluate(neighbor_position) - params.robot_radius;
        if (dist_to_obstacles <= 0) {
          continue;
        }
        const double front_distance_to_obstacles =
            distance_field.Evaluate(neighbor_front_position);
        if (front_distance_to_obstacles <= params.robot_radius) {
          continue;
        }
        obstacle_cost = kMaxDistance - dist_to_obstacles;
      }

      double path_divergence = clicked_path_field.Evaluate(neighbor_position);
      if (path_divergence >
          params.max_path_divergence * params.max_path_divergence) {
        continue;
      }
      double incremental_node_cost =
          params.travel_cost_weight * (1 /*for a single step*/) +
          params.path_divergence_weight * path_divergence +
          params.obstacle_weight * obstacle_cost +
          params.curvature_integral_weight *
              primitive.squared_curvature_integral;
      neighbor_node.cost_from_start =
          current_node.cost_from_start + incremental_node_cost;
      if (kUseNodeIntrospection) {
        neighbor_node.SetIntrospection(
            dist_to_obstacles, params.obstacle_weight * obstacle_cost,
            params.path_divergence_weight * path_divergence,
            params.curvature_integral_weight *
                primitive.squared_curvature_integral,
            params.travel_cost_weight * 1, incremental_node_cost);
      }
      open_list.push(neighbor_node);
    }
  }

  PublishIntrospection("/smooth_primitive_path", "odom", primitive_path, 0.1);

  PublishIntrospection("/expanded_path_tree", "odom", closed_list, 0.1);

  Path path_out;
  for (auto iter = std::begin(primitive_path); iter != std::end(primitive_path);
       ++iter) {
    path_out.push_back(iter->State());
    ROS_DEBUG_STREAM("[type:" << iter->previous_primitive.primitive_type
                              << ",x:" << iter->x << ",y:" << iter->y
                              << ",s:" << iter->distance_traveled
                              << ",cost2come:" << iter->cost_from_start
                              << ",cost2go:" << iter->cost_to_goal << ","
                              << iter->GetIntrospectionDescription() << "]");
  }

  return path_out;
};

}  // namespace bookbot

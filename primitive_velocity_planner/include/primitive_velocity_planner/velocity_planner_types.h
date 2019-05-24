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

#ifndef BOOKBOT_PRIMITIVE_VELOCITY_PLANNER_VELOCITY_PLANNER_TYPES_H_
#define BOOKBOT_PRIMITIVE_VELOCITY_PLANNER_VELOCITY_PLANNER_TYPES_H_

#include <trajectory_math/curve_algorithms.h>
#include <trajectory_math/trajectory.h>

#include <Eigen/Core>

namespace bookbot {

struct RobotPlanningState {
  Eigen::Vector2d Position() const { return {x, y}; }
  double time;
  double x;
  double y;
  double yaw;
  double velocity;
  double angular_velocity;
};

inline std::ostream& operator<<(std::ostream& stream,
                                const RobotPlanningState& state) {
  stream << "[t:" << state.time << " x:" << state.x << " y:" << state.y
         << " yaw:" << state.yaw << " v:" << state.velocity
         << " omega:" << state.angular_velocity << "]";
  return stream;
}

struct RobotLongitudinalPlanningState {
  double time;
  double distance;
  double velocity;
  double acceleration;
};

inline std::ostream& operator<<(std::ostream& stream,
                                const RobotLongitudinalPlanningState& state) {
  stream << "[t:" << state.time << " s:" << state.distance
         << " s_dot:" << state.velocity << " s_ddot:" << state.acceleration
         << "]";
  return stream;
}

struct ConstraintBound {
  inline bool IsSatisfied(double query_value) const {
    return query_value >= constraint_interval.min_value &&
           query_value <= constraint_interval.max_value;
  }
  Interval constraint_interval;
};

struct VelocityPlannerParams {
  // Dynamic Constraints
  ConstraintBound velocity_bounds;
  ConstraintBound desired_velocity_bounds;  // used for sampling
  ConstraintBound acceleration_bounds;
  ConstraintBound lateral_acceleration_bounds;

  // Obstacle Constraints
  double robot_radius;
  double obstacle_check_front_offset;

  // Planning Parameters
  double planning_time_horizon;
  int time_bins;
  int velocity_bins;
  double velocity_planner_cycle_time;
  double reinitialization_distance;
  int max_occupancy_grid_value;
  double max_yaw_change_for_nonzero_velocity;
  double visited_grid_distance_resolution;
  double path_end_buffer_distance;

  // Planning Cost Weights
  double travel_time_weight;
  double jerk_weight;
  double longitudinal_acceleration_weight;
  double lateral_acceleration_weight;
  double obstacle_weight;
  double desired_velocity_weight;

  // Dead Zone Parameters
  double dead_zone_distance;
  double dead_zone_velocity;
  double dead_zone_deceleration;

  // Planning mode
  bool require_perception;
  bool override_obstacles;

  // Timeouts
  double active_command_timeout;
  double desired_path_timeout;
  double odometry_timeout;
  double perception_timeout;

  // Frame Names
  std::string odometry_frame_id;

  // Obstacle Override
  double obstacle_override_timeout;
  double obstacle_override_max_velocity;
  double obstacle_override_desired_velocity;
};

}  // namespace bookbot

#endif  // BOOKBOT_PRIMITIVE_VELOCITY_PLANNER_VELOCITY_PLANNER_TYPES_H_

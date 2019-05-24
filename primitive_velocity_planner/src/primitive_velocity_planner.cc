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

#include <primitive_velocity_planner/primitive_velocity_planner.h>
#include <primitive_velocity_planner/velocity_planner_introspection.h>
#include <primitive_velocity_planner/velocity_profile_search.h>
#include <ros/ros.h>
#include <trajectory_math/curve_algorithms.h>
#include <trajectory_math/path_algorithms.h>
#include <trajectory_math/trajectory1D_algorithms.h>
#include <trajectory_math/trajectory_algorithms.h>

#include <algorithm>

namespace bookbot {

/**
 * @brief Extrapolates a robot planning state forward in time.
 * Note: this extrapolation is a very rough approximation and only suitable for
 * short timesteps.
 *
 * @param [in] current_state        The current robot state
 * @param [in] extrapolation_time   The delta time to extrapolate over
 * @return RobotPlanningState   The extrapolated state at state.time +
 *                              extrapolation_time
 */
RobotPlanningState ExtrapolateRobotState(
    const RobotPlanningState& current_state, double extrapolation_time);

/**
 * @brief Finds the closest point after a given time in a Trajectory
 *
 * @param [in] previous_trajectory     Trajectory to be matched into
 * @param [in] match_time              The query time for the matching
 * @return Trajectory::const_iterator  The closest point in the trajectory to
 *                                     occur after the match_time
 */
Trajectory::const_iterator MatchToPreviousTrajectory(
    const Trajectory& previous_trajectory, double match_time);

/**
 * @brief Extracts a prefix trajectory that includes all points on a previous
 * trajectory between the segment that includes the current time (inclusive) and
 * the matched point (exclusive)
 *
 * @param [in] previous_trajectory  The trajectory being extracted from
 * @param [in] matched_point_iter   An iterator to the previous_trajectory that
 *                                  represents the temporally matched node point
 * @param [in] current_time         The current time which we want to extend the
 *                                  prefix back to include
 * @return Trajectory   A trajectory stippet that starts at the point before the
 *                      current time and extends to the point before the
 *                      matched_point_iter
 */
Trajectory ExtractPrefixTrajectory(
    const Trajectory& previous_trajectory,
    Trajectory::const_iterator matched_point_iter, double current_time);

/**
 * @brief Generates a stopping trajectory to bring the robot to a reasonable
 * stop when it is slow enough its planned trajectory is not long enough to
 * exceed the deadzone threshold.
 *
 * @param [in] desired_path                   The fixed path to follow
 * @param [in] longitudinal_planning_state    The 1D initial state for planning
 * @param [in] params                         Deadzone parameters
 * @return Trajectory   A comfortable stopping trajectory
 */
Trajectory GenerateDeadzoneStoppingTrajectory(
    const Path& desired_path,
    const Trajectory1DPoint& longitudinal_planning_state,
    const VelocityPlannerParams& params);

Trajectory PlanVelocityProfile(const Path& desired_path,
                               const RobotPlanningState& robot_state,
                               const VelocityPlannerParams& params,
                               const Trajectory& previous_trajectory,
                               const SignedDistanceField& distance_field) {
  RobotPlanningState expected_robot_state_at_planning_time;
  double planning_time = robot_state.time + params.velocity_planner_cycle_time;
  double initial_acceleration = 0;
  Trajectory prefix;
  double prefix_end_distance_along_path = 0;
  if (previous_trajectory.empty() ||
      planning_time > previous_trajectory.back().time ||
      planning_time < previous_trajectory.front().time) {
    // No previous trajectory (or out of range), use extrapolated state
    ROS_INFO_STREAM(
        "Velocity planner reiniting due to no valid previous trajectory");
    expected_robot_state_at_planning_time =
        ExtrapolateRobotState(robot_state, params.velocity_planner_cycle_time);
  } else {
    // Plan from point after current_time + cycle_time on previous trajectory
    auto matched_trajectory_point_iter =
        MatchToPreviousTrajectory(previous_trajectory, planning_time);

    // Check if approximate control error is too big
    if (Eigen::Vector2d(matched_trajectory_point_iter->Position() -
                        robot_state.Position())
            .squaredNorm() >
        params.reinitialization_distance * params.reinitialization_distance) {
      ROS_INFO_STREAM(
          "Velocity planner reiniting due to no valid previous trajectory");
      expected_robot_state_at_planning_time = ExtrapolateRobotState(
          robot_state, params.velocity_planner_cycle_time);
    } else {
      expected_robot_state_at_planning_time.time =
          matched_trajectory_point_iter->time;
      expected_robot_state_at_planning_time.x =
          matched_trajectory_point_iter->x;
      expected_robot_state_at_planning_time.y =
          matched_trajectory_point_iter->y;
      expected_robot_state_at_planning_time.velocity =
          matched_trajectory_point_iter->velocity;
      expected_robot_state_at_planning_time.angular_velocity = 0.;
      expected_robot_state_at_planning_time.yaw =
          matched_trajectory_point_iter->yaw;
      prefix = ExtractPrefixTrajectory(
          previous_trajectory, matched_trajectory_point_iter, robot_state.time);
      prefix_end_distance_along_path =
          matched_trajectory_point_iter->distance_along_path;
    }
  }

  // Match expected robot state at planning time to desired_path spatially to
  // get longitudinal state
  const PathPoint spatially_matched_path_point =
      MatchToPath(expected_robot_state_at_planning_time.Position(),
                  std::begin(desired_path), std::end(desired_path));

  // Get robot longitudinal planning state assuming we are not deviating from
  // the desired path
  const Trajectory1DPoint longitudinal_planning_state = {
      expected_robot_state_at_planning_time.time,
      spatially_matched_path_point.distance_along_path,
      expected_robot_state_at_planning_time.velocity, initial_acceleration};

  Trajectory best_trajectory =
      PerformVelocityProfileSearch(desired_path, longitudinal_planning_state,
                                   params, previous_trajectory, distance_field);
  if (best_trajectory.empty()) {
    return best_trajectory;  // Return empty trajectory to indicate failure
  }

  // Check if the trajectory falls within the deadzone
  if (best_trajectory.front().velocity < params.dead_zone_velocity &&
      best_trajectory.back().distance_along_path -
              best_trajectory.front().distance_along_path <
          params.dead_zone_distance) {
    // Trajectory falls within deadzone - just stop at a reasonable rate
    best_trajectory = GenerateDeadzoneStoppingTrajectory(
        desired_path, longitudinal_planning_state, params);
  }

  // Update prefix distances in case underlying path has been changed
  double prefix_distance_delta =
      spatially_matched_path_point.distance_along_path -
      prefix_end_distance_along_path;
  std::for_each(std::begin(prefix), std::end(prefix),
                [prefix_distance_delta](TrajectoryPoint& pt) {
                  pt.distance_along_path += prefix_distance_delta;
                });

  // Add prefix from previous trajectory
  best_trajectory.insert(std::begin(best_trajectory), std::begin(prefix),
                         std::end(prefix));

  // Publish introspection
  PublishIntrospection("/introspection/path", params.odometry_frame_id,
                       desired_path);
  PublishIntrospection("/introspection/best_trajectory",
                       params.odometry_frame_id, best_trajectory);

  return best_trajectory;
}

RobotPlanningState ExtrapolateRobotState(
    const RobotPlanningState& current_state, double extrapolation_time) {
  RobotPlanningState extrapolated_state;
  extrapolated_state.time = current_state.time + extrapolation_time;
  extrapolated_state.yaw =
      current_state.yaw + current_state.angular_velocity * extrapolation_time;
  extrapolated_state.x = current_state.x + current_state.velocity *
                                               std::cos(current_state.yaw) *
                                               extrapolation_time;
  extrapolated_state.y = current_state.y + current_state.velocity *
                                               std::sin(current_state.yaw) *
                                               extrapolation_time;
  extrapolated_state.velocity = current_state.velocity;
  extrapolated_state.angular_velocity = current_state.angular_velocity;
  return extrapolated_state;
}

Trajectory::const_iterator MatchToPreviousTrajectory(
    const Trajectory& previous_trajectory, double match_time) {
  auto previous_trajectory_iter = std::lower_bound(
      std::begin(previous_trajectory), std::end(previous_trajectory),
      match_time, [](const TrajectoryPoint& point, double time) -> bool {
        return point.time < time;
      });
  assert(previous_trajectory_iter != std::end(previous_trajectory) &&
         "Invalid temporal match to end of trajectory");
  assert(previous_trajectory_iter != std::begin(previous_trajectory) &&
         "Invalid temporal match to beginning of trajectory");
  return previous_trajectory_iter;
}

Trajectory ExtractPrefixTrajectory(
    const Trajectory& previous_trajectory,
    Trajectory::const_iterator matched_point_iter, double current_time) {
  // Add previous points to prefix back until before current_time
  Trajectory prefix;
  auto iter = std::prev(matched_point_iter);
  for (; iter != std::begin(previous_trajectory) && iter->time > current_time;
       iter--) {
    prefix.push_back(*iter);
  }
  prefix.push_back(*iter);
  std::reverse(std::begin(prefix), std::end(prefix));
  return prefix;
}

Trajectory GenerateDeadzoneStoppingTrajectory(
    const Path& desired_path,
    const Trajectory1DPoint& longitudinal_planning_state,
    const VelocityPlannerParams& params) {
  double stop_time = MinTimeToVelocity(longitudinal_planning_state.velocity, 0.,
                                       {params.dead_zone_deceleration, 0.0});
  Trajectory1DPoint stopped_point;
  stopped_point.time = longitudinal_planning_state.time + stop_time;
  stopped_point.velocity = 0;
  stopped_point.acceleration = params.dead_zone_deceleration;
  stopped_point.distance =
      longitudinal_planning_state.distance +
      longitudinal_planning_state.velocity * stop_time +
      0.5 * params.dead_zone_deceleration * stop_time * stop_time;
  Trajectory1DPoint horizon_point;
  horizon_point.time =
      longitudinal_planning_state.time + params.planning_time_horizon;
  horizon_point.velocity = 0;
  horizon_point.distance = stopped_point.distance;
  horizon_point.acceleration = 0;
  Trajectory1D deadzone_longitudinal_trajectory = {
      longitudinal_planning_state, stopped_point, horizon_point};
  const double time_resolution =
      params.planning_time_horizon / static_cast<double>(params.time_bins);
  return GenerateTrajectoryFromVelocityProfileAndPath(
      desired_path, deadzone_longitudinal_trajectory, time_resolution,
      params.planning_time_horizon);
}

}  // namespace bookbot

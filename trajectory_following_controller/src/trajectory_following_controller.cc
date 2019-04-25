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

#include <ros/ros.h>
#include <trajectory_following_controller/trajectory_following_controller.h>
#include <trajectory_math/float_comparison.h>
#include <trajectory_math/trajectory_algorithms.h>

#include <Eigen/Geometry>
#include <limits>

namespace bookbot {

constexpr double kPurePursuitLookaheadTime = 1;
constexpr double kPurePursuitMinLookaheadDistance = 0.5;
constexpr double kLongitudinalFeedbackGain = 0.1;
constexpr double kMaxCurvatureForNonzeroVelocity = 1.5;
constexpr double kMaxLateralAcceleration = 1.0;
constexpr double kMaxPositionError = 1.0;
constexpr double kMaxAngularVelocity = 1.0;

double ComputePurePursuitCurvature(Eigen::Vector2d robot_position,
                                   double robot_yaw, Eigen::Vector2d goal_pt) {
  if (robot_position == goal_pt) {
    return 0;
  }

  Eigen::Rotation2Dd R(-robot_yaw);

  Eigen::Vector2d relative_goal_pt =
      R * Eigen::Vector2d(goal_pt - robot_position);

  double desired_curvature =
      2 * relative_goal_pt[1] / relative_goal_pt.dot(relative_goal_pt);
  return desired_curvature;
}

ControlCommand ComputeControlCommand(const Eigen::Vector2d robot_position,
                                     double robot_yaw,
                                     const Trajectory& trajectory,
                                     double current_time,
                                     ControlIntrospection* introspection) {
  // Match to trajectory spatially to compute longitudinal error
  if (ApproxEqual(trajectory.front().distance_along_path,
                  trajectory.back().distance_along_path, 0.01)) {
    introspection->matched_point = trajectory.back();
    introspection->spatially_matched_point = trajectory.back();
    introspection->lookahead_point = trajectory.back().Position();
    introspection->desired_curvature = 0.;
    return {0., 0.};
  }

  const TrajectoryPoint spatially_matched_point = MatchToTrajectory(
      robot_position, std::begin(trajectory), std::end(trajectory));

  // Interpolate trajectory based on current time
  const double current_point_time =
      std::min(current_time, trajectory.back().time);

  // Exit early if temporally or spatially past end of trajectory
  if (ApproxEqual(spatially_matched_point.distance_along_path,
                  trajectory.back().distance_along_path) ||
      ApproxEqual(current_point_time, trajectory.back().time)) {
    introspection->matched_point = trajectory.back();
    introspection->spatially_matched_point = trajectory.back();
    introspection->lookahead_point = trajectory.back().Position();
    introspection->desired_curvature = 0.;
    return {0., 0.};
  }

  // Exit early if too far away from trajectory
  if (Eigen::Vector2d(robot_position - spatially_matched_point.Position())
          .squaredNorm() > kMaxPositionError * kMaxPositionError) {
    introspection->matched_point = spatially_matched_point;
    introspection->spatially_matched_point = spatially_matched_point;
    introspection->lookahead_point = spatially_matched_point.Position();
    introspection->desired_curvature = 0.;
    return {0., 0.};
  }

  TrajectoryPoint matched_point = InterpolateTrajectoryByTime(
      current_point_time, std::begin(trajectory), std::end(trajectory));

  const double longitudinal_error = matched_point.distance_along_path -
                                    spatially_matched_point.distance_along_path;

  const double commanded_forward_velocity =
      matched_point.velocity + kLongitudinalFeedbackGain * longitudinal_error;

  const double lookahead_relative_distance =
      std::max(matched_point.velocity * kPurePursuitLookaheadTime,
               kPurePursuitMinLookaheadDistance);
  double lookahead_distance =
      spatially_matched_point.distance_along_path + lookahead_relative_distance;

  if (ApproxEqual(lookahead_distance, trajectory.back().distance_along_path) ||
      lookahead_distance > trajectory.back().distance_along_path) {
    const double distance_past_end =
        lookahead_distance - trajectory.back().distance_along_path;
    // Trajectory is too short for pure pursuit, just go in a straight line
    introspection->matched_point = matched_point;
    introspection->spatially_matched_point = spatially_matched_point;
    introspection->lookahead_point = trajectory.back().Position();
    introspection->desired_curvature = 0.;
    return {commanded_forward_velocity, 0.};
  }

  TrajectoryPoint lookahead_trajectory_point =
      InterpolateTrajectoryByDistanceAlongPath(
          lookahead_distance, std::begin(trajectory), std::end(trajectory));
  Eigen::Vector2d lookahead_point = lookahead_trajectory_point.Position();

  const double desired_curvature =
      ComputePurePursuitCurvature(robot_position, robot_yaw, lookahead_point);

  // Limit commanded_forward_velocity based on curvature
  double max_velocity_based_on_curvature =
      std::numeric_limits<double>::infinity();
  if (!ApproxZero(desired_curvature)) {
    // Limit by maximum lateral acceleration
    max_velocity_based_on_curvature =
        std::sqrt(kMaxLateralAcceleration / std::abs(desired_curvature));
  }
  if (std::abs(desired_curvature) > kMaxCurvatureForNonzeroVelocity) {
    max_velocity_based_on_curvature = 0;
  }
  const double limited_commanded_forward_velocity =
      std::min(max_velocity_based_on_curvature, commanded_forward_velocity);

  const double commanded_angular_velocity =
      std::max(limited_commanded_forward_velocity, 1.0) * desired_curvature;

  // Limit commanded_angular_velocity
  const double limited_commanded_angular_velocity = std::copysign(
      std::min(std::abs(commanded_angular_velocity), kMaxAngularVelocity),
      commanded_angular_velocity);

  introspection->matched_point = matched_point;
  introspection->spatially_matched_point = spatially_matched_point;
  introspection->lookahead_point = lookahead_point;
  introspection->desired_curvature = desired_curvature;

  return {limited_commanded_forward_velocity,
          limited_commanded_angular_velocity};
}

}  // namespace bookbot

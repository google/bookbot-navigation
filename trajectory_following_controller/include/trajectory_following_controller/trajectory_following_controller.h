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

#ifndef BOOKBOT_TRAJECTORY_FOLLOWING_CONTROLLER_H_
#define BOOKBOT_TRAJECTORY_FOLLOWING_CONTROLLER_H_

#include <trajectory_math/trajectory.h>

#include <Eigen/Core>

namespace bookbot {

struct ControlCommand {
  double forward_velocity;
  double angular_velocity;
};

struct ControlIntrospection {
  TrajectoryPoint matched_point;
  TrajectoryPoint spatially_matched_point;
  Eigen::Vector2d lookahead_point;
  double desired_curvature;
};

ControlCommand ComputeControlCommand(const Eigen::Vector2d robot_position,
                                     double robot_yaw,
                                     const Trajectory& trajectory,
                                     double current_time,
                                     ControlIntrospection* instrospection);

}  // namespace bookbot

#endif  // BOOKBOT_TRAJECTORY_FOLLOWING_CONTROLLER_H_

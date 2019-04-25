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

#ifndef BOOKBOT_TRAJECTORY_MATH_TRAJECTORY_H_
#define BOOKBOT_TRAJECTORY_MATH_TRAJECTORY_H_
#include <Eigen/Core>
#include <vector>

namespace bookbot {

struct TrajectoryPoint {
  TrajectoryPoint() = default;
  TrajectoryPoint(double time_in, double distance_along_path_in, double x_in,
                  double y_in, double yaw_in, double curvature_in,
                  double velocity_in)
      : time(time_in),
        distance_along_path(distance_along_path_in),
        x(x_in),
        y(y_in),
        yaw(yaw_in),
        curvature(curvature_in),
        velocity(velocity_in) {}
  Eigen::Vector2d Position() const { return {x, y}; }
  double time;                 //! Independant time variable
  double distance_along_path;  //! Distance along path
  double x;                    //! x position in parent frame
  double y;                    //! y position in parent frame
  double yaw;                  //! Orientation in parent frame
  double curvature;            //! Signed curvature
  double velocity;             //! Velocity
};

using Trajectory = std::vector<TrajectoryPoint>;

}  // namespace bookbot

#endif  // BOOKBOT_TRAJECTORY_MATH_TRAJECTORY_H_

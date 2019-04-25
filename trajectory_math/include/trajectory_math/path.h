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

#ifndef BOOKBOT_TRAJECTORY_MATH_PATH_H_
#define BOOKBOT_TRAJECTORY_MATH_PATH_H_
#include <Eigen/Core>
#include <vector>

namespace bookbot {

struct PathPoint {
  Eigen::Vector2d Position() const { return {x, y}; }
  double distance_along_path;  //! Distance along path (independant variable)
  double x;                    //! x position in parent frame
  double y;                    //! y position in parent frame
  double yaw;                  //! Orientation in parent frame
  double curvature;            //! Signed curvature
};

using Path = std::vector<PathPoint>;

}  // namespace bookbot

#endif  // BOOKBOT_TRAJECTORY_MATH_PATH_H_

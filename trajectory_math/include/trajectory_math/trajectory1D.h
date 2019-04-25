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

#ifndef BOOKBOT_TRAJECTORY_MATH_TRAJECTORY1D_H_
#define BOOKBOT_TRAJECTORY_MATH_TRAJECTORY1D_H_
#include <istream>
#include <vector>

namespace bookbot {

struct Trajectory1DPoint {
  Trajectory1DPoint() = default;
  Trajectory1DPoint(double time_in, double distance_in, double velocity_in,
                    double acceleration_in)
      : time(time_in),
        distance(distance_in),
        velocity(velocity_in),
        acceleration(acceleration_in) {}
  double time;          //! Independant variable
  double distance;      //! Distance
  double velocity;      //! Velocity
  double acceleration;  //! Acceleration
};

inline std::ostream& operator<<(std::ostream& stream,
                                const Trajectory1DPoint& point) {
  stream << "[t:" << point.time << " s:" << point.distance
         << " v:" << point.velocity << " a:" << point.acceleration << "]";
  return stream;
}

using Trajectory1D = std::vector<Trajectory1DPoint>;

inline std::ostream& operator<<(std::ostream& stream,
                                const Trajectory1D& trajectory) {
  for (const auto& point : trajectory) {
    stream << point;
  }
  return stream;
}

}  // namespace bookbot

#endif  // BOOKBOT_TRAJECTORY_MATH_TRAJECTORY1D_H_

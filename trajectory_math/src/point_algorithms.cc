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

#include <trajectory_math/float_comparison.h>
#include <trajectory_math/trajectory_algorithms.h>

#include <Eigen/Geometry>

namespace bookbot {
double TriangleArea(Eigen::Vector2d start_point, Eigen::Vector2d corner_point,
                    Eigen::Vector2d end_point) {
  const Eigen::Vector3d corner_to_start{start_point[0] - corner_point[0],
                                        start_point[1] - corner_point[1], 0};
  const Eigen::Vector3d corner_to_end{end_point[0] - corner_point[0],
                                      end_point[1] - corner_point[1], 0};
  return 0.5 *
         std::abs(Eigen::Vector3d(corner_to_start.cross(corner_to_end))[2]);
}

double ApproximateCurvatureMagnitude(Eigen::Vector2d start_point,
                                     Eigen::Vector2d corner_point,
                                     Eigen::Vector2d end_point) {
  const double area = TriangleArea(start_point, corner_point, end_point);
  const double start_to_corner =
      Eigen::Vector2d(start_point - corner_point).squaredNorm();
  const double start_to_end =
      Eigen::Vector2d(start_point - end_point).squaredNorm();
  const double corner_to_end =
      Eigen::Vector2d(corner_point - end_point).squaredNorm();
  const double normalization = start_to_corner * start_to_end * corner_to_end;
  if (ApproxZero(normalization)) {
    return 0.;
  }
  return 4 * area / std::sqrt(normalization);
}

}  // namespace bookbot

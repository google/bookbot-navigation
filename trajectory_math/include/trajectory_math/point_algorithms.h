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

#ifndef AREA120_SIDEWALKER_TRAJECTORY_MATH_POINT_ALGORITHMS_H_
#define AREA120_SIDEWALKER_TRAJECTORY_MATH_POINT_ALGORITHMS_H_
#include <Eigen/Core>

namespace bookbot {

/**
 * @brief Computes area of the triangle formed by three successive points.
 *
 *                 * start_point
 *                /
 *  corner_point *-----* end_point
 */
double TriangleArea(Eigen::Vector2d start_point, Eigen::Vector2d corner_point,
                    Eigen::Vector2d end_point);

/**
 * @brief Computes an approximation of the curvature magnitude at the center
 * point of three sequential points. This approximation is based on computing
 * the radius of the circle that circumscribes the three points.
 */
double ApproximateCurvatureMagnitude(Eigen::Vector2d start_point,
                                     Eigen::Vector2d corner_point,
                                     Eigen::Vector2d end_point);

}  // namespace bookbot

#endif  // AREA120_SIDEWALKER_TRAJECTORY_MATH_POINT_ALGORITHMS_H_

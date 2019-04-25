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

#ifndef BOOKBOT_TRAJECTORY_MATH_CUBIC_SPIRAL_H_
#define BOOKBOT_TRAJECTORY_MATH_CUBIC_SPIRAL_H_

#include <trajectory_math/path.h>

namespace bookbot {

/**
 * @brief Computes an interpolated path point for a cubic spiral with the given
 * start and end parameters. The cubic spiral is represented as a cubic hermite
 * polynomial for yaw angle as a function of distance along the path:
 *
 *    \theta(s) = h00*\theta0 + h10*\kappa0 + h01*\theta1 + h11*\kappa1
 *
 * where \theta0, \kappa0 are the starting yaw angle and curvature, \theta1,
 * \kappa1 are the end yaw angle and curvature, s is distance along the path,
 * and h00, h10, h01, h11 are the cubic hermite basis functions
 * (https://en.wikipedia.org/wiki/Cubic_Hermite_spline)
 *
 * This polynomial fully defines a spatial path where dx, dy are given by the
 * Fresnel integrals:
 *
 * dx = \int_{0}^{s} cos(\theta(s))ds
 * dy = \int_{0}^{s} sin(\theta(s))ds
 *
 * @param [in] segment_start_point              The start point of the cubic
 *                                              spiral segment
 * @param [in] segment_length                   The length of the cubic spiral
 *                                              segment such that the end point
 *                                              distance_along_path =
 *                                              start distance_along_path +
 *                                              segment_length
 * @param [in] segment_end_yaw                  The ending yaw angle of the
 *                                              cubic spiral segment
 * @param [in] segment_end_curvature            The ending curvature of the
 *                                              cubic spiral segment
 * @param [in] segment_interpolation_distance   The distance to interpolate
 *                                              along the cubic spiral segment
 * @return PathPoint  The interpolated path point along the cubic spiral segment
 */
PathPoint InterpolateCubicSpiralSegmentByDistanceAlongPath(
    const PathPoint& segment_start_point, double segment_length,
    double segment_end_yaw, double segment_end_curvature,
    double segment_interpolation_distance);

/**
 * @brief Computes the integral of the squared curvature along a cubic spiral
 * segment.
 *
 * @param [in] segment_start_point     The start point of the cubic spiral
 *                                     segment
 * @param [in] segment_length          The length of the cubic spiral segment
 * @param [in] segment_end_yaw         The ending yaw angle of the cubic spiral
 *                                     segment
 * @param [in] segment_end_curvature   The ending curvature of the cubic spiral
 *                                     segment
 * @return double  The integral of the squared curvature along the cubic spiral
 *                 segment
 */
double EvaluateCubicSpiralSegmentSquaredCurvatureIntegral(
    const PathPoint& segment_start_point, double segment_length,
    double segment_end_yaw, double segment_end_curvature);

}  // namespace bookbot

#endif  // BOOKBOT_TRAJECTORY_MATH_CUBIC_SPIRAL_H_

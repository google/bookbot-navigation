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

#ifndef BOOKBOT_TRAJECTORY_MATH_CUBIC_HERMITE_INTERPOLATION_H_
#define BOOKBOT_TRAJECTORY_MATH_CUBIC_HERMITE_INTERPOLATION_H_
#include <Eigen/Core>
#include <vector>

namespace bookbot {

/**
 * @brief Cubic Hermite Interpolation between two end points of a cubic
 * segment defined by function x(t) = a*t^3 + b*t^2 + c*t + d.
 *
 * @param [in] t_interpolation   Interpolation value of independant variable t.
 * @param [in] t_start           Independant variable t value at start point.
 * @param [in] x_start           Value of x(t) at t_start.
 * @param [in] x_dot_start       Value of first derivative dx(t)/dt at t_start.
 * @param [in] t_end             Independant variable t value at end point.
 * @param [in] x_end             Value of x(t) at t_end.
 * @param [in] x_dot_end         Value of first derivative dx(t)/dt at t_end.
 * @return Value of x(t_interpolation).
 */
double CubicHermiteInterpolate(double t_interpolation, double t_start,
                               double x_start, double x_dot_start, double t_end,
                               double x_end, double x_dot_end);

/**
 * @brief Returns first derivative of Cubic Hermite Interpolation between two
 * end points of a cubic segment. See CubicHermitInterpolate for parameter
 * description.
 **/
double CubicHermiteInterpolateDerivative(double t_interpolation, double t_start,
                                         double x_start, double x_dot_start,
                                         double t_end, double x_end,
                                         double x_dot_end);

/**
 * @brief Returns second derivative of Cubic Hermite Interpolation between two
 * end points of a cubic segment. See CubicHermitInterpolate for parameter
 * description.
 **/
double CubicHermiteInterpolateSecondDerivative(double t_interpolation,
                                               double t_start, double x_start,
                                               double x_dot_start, double t_end,
                                               double x_end, double x_dot_end);

}  // namespace bookbot

#endif  // BOOKBOT_TRAJECTORY_MATH_CUBIC_HERMITE_INTERPOLATION_H_

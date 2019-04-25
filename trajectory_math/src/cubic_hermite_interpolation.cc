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

#include <trajectory_math/cubic_hermite_interpolation.h>
#include <trajectory_math/float_comparison.h>

namespace bookbot {

double CubicHermiteInterpolate(double t_interpolation, double t_start,
                               double x_start, double x_dot_start, double t_end,
                               double x_end, double x_dot_end) {
  assert(ApproxEqual(t_interpolation, t_start) || t_interpolation > t_start);
  assert(ApproxEqual(t_interpolation, t_end) || t_interpolation < t_end);
  if (ApproxEqual(t_start, t_end)) {
    return x_start;
  }
  assert(t_end > t_start);
  const double lambda = t_end - t_start;
  const double u = (t_interpolation - t_start) / lambda;
  const double u_squared = u * u;
  const double u_cubed = u_squared * u;

  // compute the shape functions
  const double H1 = 2 * u_cubed - 3 * u_squared + 1;
  const double H2 = u_cubed - 2 * u_squared + u;
  const double H3 = -2 * u_cubed + 3 * u_squared;
  const double H4 = u_cubed - u_squared;

  // return the interpolated value
  const double interpolated_value = H1 * x_start + H2 * lambda * x_dot_start +
                                    H3 * x_end + H4 * lambda * x_dot_end;
  return interpolated_value;
}

double CubicHermiteInterpolateDerivative(double t_interpolation, double t_start,
                                         double x_start, double x_dot_start,
                                         double t_end, double x_end,
                                         double x_dot_end) {
  assert(ApproxEqual(t_interpolation, t_start) || t_interpolation > t_start);
  assert(ApproxEqual(t_interpolation, t_end) || t_interpolation < t_end);
  if (ApproxEqual(t_start, t_end)) {
    return x_dot_start;
  }
  assert(t_end > t_start);
  const double lambda = t_end - t_start;
  const double u = (t_interpolation - t_start) / lambda;
  const double u_squared = u * u;

  // compute the shape function derivatives
  const double H1_prime = 6 * u_squared - 6 * u;
  const double H2_prime = 3 * u_squared - 4 * u + 1;
  const double H3_prime = -6 * u_squared + 6 * u;
  const double H4_prime = 3 * u_squared - 2 * u;

  // return the interpolated value
  const double interpolated_value_derivative =
      H1_prime * x_start + H2_prime * lambda * x_dot_start + H3_prime * x_end +
      H4_prime * lambda * x_dot_end;
  return interpolated_value_derivative / lambda;
}

double CubicHermiteInterpolateSecondDerivative(double t_interpolation,
                                               double t_start, double x_start,
                                               double x_dot_start, double t_end,
                                               double x_end, double x_dot_end) {
  assert(ApproxEqual(t_interpolation, t_start) || t_interpolation > t_start);
  assert(ApproxEqual(t_interpolation, t_end) || t_interpolation < t_end);
  if (ApproxEqual(t_start, t_end)) {
    return x_dot_start;
  }
  assert(t_end > t_start);
  const double lambda = t_end - t_start;
  const double u = (t_interpolation - t_start) / lambda;

  // compute the shape function second derivatives
  const double H1_pprime = 12 * u - 6;
  const double H2_pprime = 6 * u - 4;
  const double H3_pprime = -12 * u + 6;
  const double H4_pprime = 6 * u - 2;

  // return the interpolated value
  const double interpolated_value_second_derivative =
      H1_pprime * x_start + H2_pprime * lambda * x_dot_start +
      H3_pprime * x_end + H4_pprime * lambda * x_dot_end;
  return interpolated_value_second_derivative / (lambda * lambda);
}

}  // namespace bookbot

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

#ifndef BOOKBOT_TRAJECTORY_MATH_GAUSS_LEGENDRE_INTEGRATION_H
#define BOOKBOT_TRAJECTORY_MATH_GAUSS_LEGENDRE_INTEGRATION_H
#include <Eigen/Core>
#include <vector>

namespace bookbot {

/**
 * @brief Computes a close approximation of the definite integral from -1 to 1
 * of the provided query_function using Gauss-Legendre Quadrature of order n=12.
 *
 * Gauss-Legendre Quadrature:
 *
 * \int_{-1}^{1} = \sum_{i=1}^{12} w_i * query_function(a_i)
 *
 * where w_i and a_i are fixed weights and abscissae
 *
 * @param [in] query_function   The function that is being integrated
 * @return double               The definite integral over the range -1 to 1
 */
double GaussLegendreIntegrate(
    const std::function<double(double)>& query_function);

/**
 * @brief Computes a close approximateion of the definite integral over the
 * range x_start to x_end of the provided query_function using Gauss-Legendre
 * Quadratrue of order n=12.
 *
 * Gauss-Legendre Quadrature:
 *
 * \int_{l}^{u} =
 *   (u - l)/2 * \sum_{i=1}^{12} w_i * query_function((u-l)/2*a_i + (u+l)/2)
 *
 * where w_i and a_i are fixed weights and abscissae
 *
 * @param [in] query_function   The function that is being integrated
 * @param [in] x_start          Start of the definite inegral integration range
 * @param [in] x_end            End of the definite integral integration range
 * @return double               The definite integral over the range x_start to
 * x_end
 */
double GaussLegendreIntegrate(
    const std::function<double(double)>& query_function, double x_start,
    double x_end);

}  // namespace bookbot

#endif  // BOOKBOT_TRAJECTORY_MATH_GAUSS_LEGENDRE_INTEGRATION_H

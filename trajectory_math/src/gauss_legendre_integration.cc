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

#include <trajectory_math/gauss_legendre_integration.h>

#include <array>

namespace bookbot {

constexpr std::size_t integration_order = 12;

constexpr std::array<double, integration_order> gauss_legendre_weights = {
    0.2491470458134028, 0.2491470458134028, 0.2334925365383548,
    0.2334925365383548, 0.2031674267230659, 0.2031674267230659,
    0.1600783285433462, 0.1600783285433462, 0.1069393259953184,
    0.1069393259953184, 0.0471753363865118, 0.0471753363865118};

constexpr std::array<double, integration_order> gauss_legendre_abscissa = {
    -0.1252334085114689, 0.1252334085114689,  -0.3678314989981802,
    0.3678314989981802,  -0.5873179542866175, 0.5873179542866175,
    -0.7699026741943047, 0.7699026741943047,  -0.9041172563704749,
    0.9041172563704749,  -0.9815606342467192, 0.9815606342467192};

double GaussLegendreIntegrate(
    const std::function<double(double)>& query_function) {
  double sum = 0;
  for (int i = 0; i < integration_order; ++i) {
    sum +=
        gauss_legendre_weights[i] * query_function(gauss_legendre_abscissa[i]);
  }
  return sum;
}

double GaussLegendreIntegrate(
    const std::function<double(double)>& query_function, double x_start,
    double x_end) {
  auto factor1 = (x_end - x_start) / 2;
  auto factor2 = (x_end + x_start) / 2;
  auto normalized_query_function = [&query_function, factor1,
                                    factor2](double x) -> double {
    return query_function(factor1 * x + factor2);
  };
  return factor1 * GaussLegendreIntegrate(normalized_query_function);
}
}  // namespace bookbot

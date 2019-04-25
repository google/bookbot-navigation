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
#include <trajectory_math/cubic_spiral.h>
#include <trajectory_math/gauss_legendre_integration.h>

namespace bookbot {

PathPoint InterpolateCubicSpiralSegmentByDistanceAlongPath(
    const PathPoint& segment_start_point, double segment_length,
    double segment_end_yaw, double segment_end_curvature,
    double segment_interpolation_distance) {
  auto yaw_polynomial =
      [&segment_start_point, segment_length, segment_end_yaw,
       segment_end_curvature](double distance_along_path) -> double {
    return CubicHermiteInterpolate(
        distance_along_path, 0, segment_start_point.yaw,
        segment_start_point.curvature, segment_length, segment_end_yaw,
        segment_end_curvature);
  };

  auto curvature_polynomial =
      [&segment_start_point, segment_length, segment_end_yaw,
       segment_end_curvature](double distance_along_path) -> double {
    return CubicHermiteInterpolateDerivative(
        distance_along_path, 0, segment_start_point.yaw,
        segment_start_point.curvature, segment_length, segment_end_yaw,
        segment_end_curvature);
  };

  const double dx = GaussLegendreIntegrate(
      [&yaw_polynomial](double distance) -> double {
        return std::cos(yaw_polynomial(distance));
      },
      0, segment_interpolation_distance);

  const double dy = GaussLegendreIntegrate(
      [&yaw_polynomial](double distance) -> double {
        return std::sin(yaw_polynomial(distance));
      },
      0, segment_interpolation_distance);

  PathPoint evaluated_point;
  evaluated_point.x = segment_start_point.x + dx;
  evaluated_point.y = segment_start_point.y + dy;
  evaluated_point.yaw = yaw_polynomial(segment_interpolation_distance);
  evaluated_point.distance_along_path =
      segment_start_point.distance_along_path + segment_interpolation_distance;
  evaluated_point.curvature =
      curvature_polynomial(segment_interpolation_distance);
  return evaluated_point;
}

double EvaluateCubicSpiralSegmentSquaredCurvatureIntegral(
    const PathPoint& segment_start_point, double segment_length,
    double segment_end_yaw, double segment_end_curvature) {
  auto curvature_polynomial =
      [&segment_start_point, segment_length, segment_end_yaw,
       segment_end_curvature](double distance_along_path) -> double {
    return CubicHermiteInterpolateDerivative(
        distance_along_path, 0, segment_start_point.yaw,
        segment_start_point.curvature, segment_length, segment_end_yaw,
        segment_end_curvature);
  };

  return GaussLegendreIntegrate(
      [&curvature_polynomial](double distance) -> double {
        return std::pow(curvature_polynomial(distance), 2);
      },
      0, segment_length);
}

}  // namespace bookbot

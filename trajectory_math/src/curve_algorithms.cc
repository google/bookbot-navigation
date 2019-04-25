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

#include <trajectory_math/trajectory_algorithms.h>

namespace bookbot {

double AngleDifference(double first_angle, double second_angle) {
  const double diff = std::fmod(second_angle - first_angle, 2 * M_PI);
  return std::abs(diff) > M_PI ? diff - std::copysign(2 * M_PI, diff) : diff;
}

double LinearInterpolation(double start, double end, double interp_fraction) {
  return start + interp_fraction * (end - start);
}

double LinearAngleInterpolation(double angle_start, double angle_end,
                                double interp_fraction) {
  return angle_start +
         interp_fraction * AngleDifference(angle_start, angle_end);
}

double SquaredDistanceToLineSegment(
    Eigen::Vector2d query_point, Eigen::Vector2d first_line_point,
    Eigen::Vector2d second_line_point,
    double* normalized_matched_distance_between_line_points) {
  const Eigen::Vector2d relative_line_segment =
      second_line_point - first_line_point;
  const double squared_relative_line_segment_dist =
      relative_line_segment.dot(relative_line_segment);
  const Eigen::Vector2d relative_query_point = query_point - first_line_point;
  const double normalized_projected_query =
      relative_line_segment.dot(relative_query_point) /
      squared_relative_line_segment_dist;
  Eigen::Vector2d closest_point;
  double closest_point_interp_fraction;
  if (normalized_projected_query <= 0.) {
    // first_line_point is closest
    closest_point_interp_fraction = 0;
    closest_point = first_line_point;
  } else if (normalized_projected_query >= 1) {
    // second_line_point is closest
    closest_point_interp_fraction = 1;
    closest_point = second_line_point;
  } else {
    // closest point is on interior of line segment
    closest_point_interp_fraction = normalized_projected_query;
    closest_point = first_line_point +
                    relative_line_segment * closest_point_interp_fraction;
  }

  if (normalized_matched_distance_between_line_points) {
    *normalized_matched_distance_between_line_points =
        closest_point_interp_fraction;
  }

  return Eigen::Vector2d(query_point - closest_point).squaredNorm();
}

}  // namespace bookbot

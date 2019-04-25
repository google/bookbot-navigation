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

#ifndef BOOKBOT_TRAJECTORY_MATH_CURVE_ALGORITHMS_H_
#define BOOKBOT_TRAJECTORY_MATH_CURVE_ALGORITHMS_H_
#include <Eigen/Core>

namespace bookbot {

struct Interval {
  double min_value;
  double max_value;
};

double AngleDifference(double first_angle, double second_angle);

double LinearInterpolation(double start, double end, double interp_fraction);

double LinearAngleInterpolation(double angle_start, double angle_end,
                                double interp_fraction);

double SquaredDistanceToLineSegment(
    Eigen::Vector2d query_point, Eigen::Vector2d first_line_point,
    Eigen::Vector2d second_line_point,
    double* normalized_matched_distance_between_line_points = nullptr);

/// \brief MatchToCurve performs a spatial matching of a query point to a curve.
/// \tparam CurveIteratorType  Container forward iterator whose value_type is a
/// spatial point with a Position() member function that returns an
/// Eigen::Vector2d representation of the {x,y} position in 2D.
/// \return Pair of CurveIteratorTypes representing the start and end point of
/// the closest segment
template <typename CurveIteratorType>
std::pair<CurveIteratorType, CurveIteratorType> FindClosestSegment(
    Eigen::Vector2d query_point, CurveIteratorType curve_begin,
    CurveIteratorType curve_end) {
  if (curve_begin == curve_end || std::next(curve_begin) == curve_end) {
    return {curve_begin, curve_begin};  // only 0 or 1 points
  }
  // find bracketing CurvePointTypes assuming linear interpolation
  double min_squared_distance = std::numeric_limits<double>::infinity();
  CurveIteratorType min_segment_iterator = curve_end;
  for (auto it = curve_begin; it != std::prev(curve_end) && it != curve_end;
       ++it) {
    const double squared_distance = SquaredDistanceToLineSegment(
        query_point, it->Position(), std::next(it)->Position());
    if (squared_distance < min_squared_distance) {
      min_squared_distance = squared_distance;
      min_segment_iterator = it;
    }
  }

  assert(min_segment_iterator != curve_end);
  return {min_segment_iterator, std::next(min_segment_iterator)};
}

}  // namespace bookbot

#endif  // BOOKBOT_TRAJECTORY_MATH_CURVE_ALGORITHMS_H_

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

#include <trajectory_math/cubic_spiral.h>
#include <trajectory_math/curve_algorithms.h>
#include <trajectory_math/float_comparison.h>
#include <trajectory_math/path_algorithms.h>

namespace bookbot {

PathPoint InterpolateBetweenPathPointsByDistanceAlongPath(
    double distance_along_path_interp, const PathPoint& point_before,
    const PathPoint& point_after) {
  assert(distance_along_path_interp <= point_after.distance_along_path &&
         distance_along_path_interp >= point_before.distance_along_path);
  if (ApproxEqual(point_before.distance_along_path,
                  point_after.distance_along_path)) {
    return point_before;
  }
  const double s_interp_fraction =
      (distance_along_path_interp - point_before.distance_along_path) /
      (point_after.distance_along_path - point_before.distance_along_path);
  PathPoint interpolated_pt;
  interpolated_pt.distance_along_path = distance_along_path_interp;
  interpolated_pt.x =
      LinearInterpolation(point_before.x, point_after.x, s_interp_fraction);
  interpolated_pt.y =
      LinearInterpolation(point_before.y, point_after.y, s_interp_fraction);
  interpolated_pt.yaw = LinearAngleInterpolation(
      point_before.yaw, point_after.yaw, s_interp_fraction);
  interpolated_pt.curvature = LinearInterpolation(
      point_before.curvature, point_after.curvature, s_interp_fraction);
  return interpolated_pt;
}

PathPoint InterpolatePathByDistanceAlongPath(double distance_along_path_interp,
                                             Path::const_iterator path_begin,
                                             Path::const_iterator path_end) {
  assert(path_begin != path_end);
  if (std::next(path_begin) == path_end) {
    return *path_begin;
  }
  auto lower_bound_it = std::lower_bound(
      path_begin, path_end, distance_along_path_interp,
      [](const PathPoint& path_point, const double distance) -> bool {
        return path_point.distance_along_path < distance;
      });
  if (lower_bound_it == path_end) {
    return *std::prev(path_end);
  }
  if (lower_bound_it != path_begin) {
    lower_bound_it = std::prev(lower_bound_it);
  }

  return InterpolateBetweenPathPointsByDistanceAlongPath(
      distance_along_path_interp, *lower_bound_it, *std::next(lower_bound_it));
}

PathPoint InterpolateCubicSpiralBetweenPathPointsByDistanceAlongPath(
    double distance_along_path_interp, const PathPoint& point_before,
    const PathPoint& point_after) {
  auto pt = InterpolateCubicSpiralSegmentByDistanceAlongPath(
      point_before,
      point_after.distance_along_path - point_before.distance_along_path,
      point_after.yaw, point_after.curvature,
      distance_along_path_interp - point_before.distance_along_path);
  return pt;
}

PathPoint InterpolateCubicSpiralPathByDistanceAlongPath(
    double distance_along_path_interp, Path::const_iterator path_begin,
    Path::const_iterator path_end) {
  assert(path_begin != path_end);
  if (std::next(path_begin) == path_end) {
    return *path_begin;
  }
  auto lower_bound_it = std::lower_bound(
      path_begin, path_end, distance_along_path_interp,
      [](const PathPoint& path_point, const double distance) -> bool {
        return path_point.distance_along_path < distance;
      });
  if (lower_bound_it == path_end) {
    return *std::prev(path_end);
  }
  if (lower_bound_it != path_begin) {
    lower_bound_it = std::prev(lower_bound_it);
  }

  return InterpolateCubicSpiralBetweenPathPointsByDistanceAlongPath(
      distance_along_path_interp, *lower_bound_it, *std::next(lower_bound_it));
}

PathPoint MatchBetweenPathPoints(Eigen::Vector2d query_point,
                                 const PathPoint& start_point,
                                 const PathPoint& end_point) {
  double interp_fraction;
  auto dist =
      SquaredDistanceToLineSegment(query_point, start_point.Position(),
                                   end_point.Position(), &interp_fraction);

  double distance_along_path_interp =
      start_point.distance_along_path +
      interp_fraction *
          (end_point.distance_along_path - start_point.distance_along_path);
  return InterpolateBetweenPathPointsByDistanceAlongPath(
      distance_along_path_interp, start_point, end_point);
}

PathPoint MatchToPath(Eigen::Vector2d query_point,
                      Path::const_iterator path_begin,
                      Path::const_iterator path_end) {
  assert(path_begin != path_end && std::next(path_begin) != path_end);
  auto bracketing_points =
      FindClosestSegment(query_point, path_begin, path_end);
  return MatchBetweenPathPoints(query_point, *bracketing_points.first,
                                *bracketing_points.second);
}
}  // namespace bookbot

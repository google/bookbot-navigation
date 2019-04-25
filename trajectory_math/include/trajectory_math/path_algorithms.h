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

#ifndef BOOKBOT_TRAJECTORY_MATH_PATH_ALGORITHMS_H_
#define BOOKBOT_TRAJECTORY_MATH_PATH_ALGORITHMS_H_
#include <trajectory_math/path.h>

#include <Eigen/Core>
#include <vector>

namespace bookbot {

/**
 * @brief Computes an interpolated PathPoint between two existing PathPoints at
 * a given interpolation distance.
 * @note This function will return the closest point on the path segment, this
 * means it will return point_before or point_after if the
 * interpolation_distance is not contained between the points.
 *
 * @param distance_along_path_interp  Path distance to be interpolated
 * @param point_before                The start point of the path segment
 * @param point_after                 The end point of the path segment
 * @return Closest interpolated PathPoint
 */
PathPoint InterpolateBetweenPathPointsByDistanceAlongPath(
    double distance_along_path_interp, const PathPoint& point_before,
    const PathPoint& point_after);

/**
 * @brief Computes the interpolated PathPoint between two Path iterators
 * specifying a given path or subpath.
 * @note This function will return the closest point on the path, this means it
 * will return the first or last point in the path if the interpolation_distance
 * is not contained within the path.
 *
 * @param distance_along_path_interp  Path distance to be interpolated
 * @param path_begin                  The begin iterator for the path
 * @param path_end                    The end iterator for the path
 * @return Closest interpolated PathPoint
 */
PathPoint InterpolatePathByDistanceAlongPath(double distance_along_path_interp,
                                             Path::const_iterator path_begin,
                                             Path::const_iterator path_end);

/**
 * @brief Computes an interpolated PathPoint between two existing PathPoints at
 * a given interpolation distance assuming that the yaw and curvature of the two
 * path points define a cubic spiral between them.
 * @note This function will return the closest point on the path segment, this
 * means it will return point_before or point_after if the
 * interpolation_distance is not contained between the points.
 *
 * @param distance_along_path_interp  Path distance to be interpolated
 * @param point_before                The start point of the path segment
 * @param point_after                 The end point of the path segment
 * @return Closest interpolated PathPoint on the cubic spiral curve
 */
PathPoint InterpolateCubicSpiralBetweenPathPointsByDistanceAlongPath(
    double distance_along_path_interp, const PathPoint& point_before,
    const PathPoint& point_after);

/**
 * @brief Computes the interpolated PathPoint between two Path iterators
 * specifying a given path or subpath assuming the points form a cubic spiral
 * spline where each sequential pair of points defines a cubic spiral path
 * segment.
 * @note This function will return the closest point on the path, this means it
 * will return the first or last point in the path if the interpolation_distance
 * is not contained within the path.
 *
 * @param distance_along_path_interp  Path distance to be interpolated
 * @param path_begin                  The begin iterator for the path
 * @param path_end                    The end iterator for the path
 * @return Closest interpolated PathPoint
 */
PathPoint InterpolateCubicSpiralPathByDistanceAlongPath(
    double distance_along_path_interp, Path::const_iterator path_begin,
    Path::const_iterator path_end);

/**
 * @brief Computes the closest point on a given path segment to the query point.
 *
 * @param query_point   Point to be matched to the path segment.
 * @param start_point   The start point of the path segment.
 * @param end_point     The end point of the path segment.
 * @return Cosest PathPoint on the path segment.
 */
PathPoint MatchBetweenPathPoints(Eigen::Vector2d query_point,
                                 const PathPoint& start_point,
                                 const PathPoint& end_point);

/**
 * @brief Computes the closest point on a given path to the query point.
 *
 * @param query_point   Point to be matched to the path.
 * @param path_begin    The begin iterator for the path
 * @param path_end      The end iterator for the path
 * @return Cosest PathPoint on the path.
 */
PathPoint MatchToPath(Eigen::Vector2d query_point,
                      Path::const_iterator path_begin,
                      Path::const_iterator path_end);

}  // namespace bookbot

#endif  // BOOKBOT_TRAJECTORY_MATH_PATH_ALGORITHMS_H_

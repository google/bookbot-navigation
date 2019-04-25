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

#ifndef BOOKBOT_TRAJECTORY_MATH_TRAJECTORY_ALGORITHMS_H_
#define BOOKBOT_TRAJECTORY_MATH_TRAJECTORY_ALGORITHMS_H_
#include <trajectory_math/path.h>
#include <trajectory_math/trajectory.h>
#include <trajectory_math/trajectory1D.h>

#include <Eigen/Core>
#include <vector>

namespace bookbot {

/// \return Time at interpolation distance
double ConstantAccelerationInterpolationByDistanceAlongPath(
    double time_start, double distance_along_path_start, double velocity_start,
    double time_end, double distance_along_path_end, double velocity_end,
    double distance_along_path_interp,
    double* interpolated_acceleration = nullptr);

/// \return Distance along path at interpolation time
double ConstantAccelerationInterpolationByTime(
    double time_start, double distance_along_path_start, double velocity_start,
    double time_end, double distance_along_path_end, double velocity_end,
    double time_interp, double* interpolated_acceleration = nullptr);

/**
 * @brief Computes an interpolated TrajectoryPoint between two existing
 * TrajectoryPoints at a given interpolation distance.
 * @note This function will return the closest point on the trajectory segment,
 * this means it will return point_before or point_after if the
 * interpolation_distance is not contained between the points.
 *
 * @param distance_along_path_interp  Distance to be interpolated
 * @param point_before                The start point of the trajectory segment
 * @param point_after                 The end point of the trajectory segment
 * @return Closest interpolated TrajectoryPoint
 */
TrajectoryPoint InterpolateBetweenTrajectoryPointsByDistanceAlongPath(
    double distance_along_path_interp, const TrajectoryPoint& point_before,
    const TrajectoryPoint& point_after,
    double* interpolated_acceleration = nullptr);

/**
 * @brief Computes an interpolated TrajectoryPoint between two existing
 * TrajectoryPoints at a given interpolation time.
 * @note This function will return the closest point on the trajectory segment,
 * this means it will return point_before or point_after if the
 * interpolation time is not contained between the points.
 *
 * @param time_interp   Time to be interpolated
 * @param point_before  The start point of the trajectory segment
 * @param point_after   The end point of the trajectory segment
 * @return Closest interpolated TrajectoryPoint
 */
TrajectoryPoint InterpolateBetweenTrajectoryPointsByTime(
    double time_interp, const TrajectoryPoint& point_before,
    const TrajectoryPoint& point_after,
    double* interpolated_acceleration = nullptr);

/**
 * @brief Computes the interpolated TrajectoryPoint between two Trajectory
 * iterators for a given interpolation distance.
 * @note This function will return the closest point on the Trajectory, this
 * means it will return the first or last point in the Trajectory if the
 * distance_along_path_interp is not contained within the Trajectory.
 *
 * @param distance_along_path_interp  Distance to be interpolated
 * @param path_begin                  The begin iterator for the trajectory
 * @param path_end                    The end iterator for the trajectory
 * @return Closest interpolated TrajectoryPoint
 */
TrajectoryPoint InterpolateTrajectoryByDistanceAlongPath(
    double distance_along_path_interp,
    Trajectory::const_iterator trajectory_begin,
    Trajectory::const_iterator trajectory_end,
    double* interpolated_acceleration = nullptr);

/**
 * @brief Computes the interpolated TrajectoryPoint between two Trajectory
 * iterators for a given interpolation time.
 * @note This function will return the closest point on the Trajectory, this
 * means it will return the first or last point in the Trajectory if the
 * time_interp is not contained within the Trajectory.
 *
 * @param time_interp  Time to be interpolated
 * @param path_begin   The begin iterator for the trajectory
 * @param path_end     The end iterator for the trajectory
 * @return Closest interpolated TrajectoryPoint
 */
TrajectoryPoint InterpolateTrajectoryByTime(
    double time_interp, Trajectory::const_iterator trajectory_begin,
    Trajectory::const_iterator trajectory_end,
    double* interpolated_acceleration = nullptr);

/**
 * @brief Computes the closest point on a given Trajectory segment to the query
 * point.
 *
 * @param query_point   Point to be matched to the trajectory segment.
 * @param start_point   The start point of the trajectory segment.
 * @param end_point     The end point of the trajectory segment.
 * @return Cosest TrajectoryPoint on the trajectory segment.
 */
TrajectoryPoint MatchBetweenTrajectoryPoints(Eigen::Vector2d query_point,
                                             const TrajectoryPoint& start_point,
                                             const TrajectoryPoint& end_point);

/**
 * @brief Computes the closest point on a given Trajectory to the query point.
 *
 * @param query_point   Point to be matched to the trajectory.
 * @param trajectory_begin    The begin iterator for the trajectory
 * @param trajectory_end      The end iterator for the trajectory
 * @return Cosest TrajectoryPoint on the path.
 */
TrajectoryPoint MatchToTrajectory(Eigen::Vector2d query_point,
                                  Trajectory::const_iterator trajectory_begin,
                                  Trajectory::const_iterator trajectory_end);

/**
 * @brief Generates a Trajectory from a velocity profile defined as a
 * Trajectory1D and a Path.
 *
 * @param path               The spatial path the trajectory should follow
 * @param velocity_profile   The temporal profile the trajectory should follow
 * @param time_resolution    The time resolution at which the resulting
 *                           trajectory should be discretized
 * @param time_horizon       How long of a trajectory to generate
 * @return The generated Trajectory
 */
Trajectory GenerateTrajectoryFromVelocityProfileAndPath(
    const Path& path, const Trajectory1D& velocity_profile,
    double time_resolution, double time_horizon);

}  // namespace bookbot

#endif  // BOOKBOT_TRAJECTORY_MATH_TRAJECTORY_ALGORITHMS_H_

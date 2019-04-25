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

#include <trajectory_math/curve_algorithms.h>
#include <trajectory_math/float_comparison.h>
#include <trajectory_math/path_algorithms.h>
#include <trajectory_math/trajectory1D_algorithms.h>
#include <trajectory_math/trajectory_algorithms.h>

namespace bookbot {

double ConstantAccelerationInterpolationByDistanceAlongPath(
    double time_start, double distance_along_path_start, double velocity_start,
    double time_end, double distance_along_path_end, double velocity_end,
    double distance_along_path_interp, double* interpolated_acceleration) {
  assert(distance_along_path_start <= distance_along_path_interp &&
         distance_along_path_interp <= distance_along_path_end);
  if (ApproxEqual(distance_along_path_start, distance_along_path_end) ||
      ApproxEqual(time_start, time_end)) {
    if (interpolated_acceleration) {
      *interpolated_acceleration = 0;
    }
    return time_start;
  }
  double a = (velocity_end - velocity_start) / (time_end - time_start);
  double v_interp = std::sqrt(
      velocity_start * velocity_start +
      2 * a * (distance_along_path_interp - distance_along_path_start));
  double time_interp = time_start + (v_interp - velocity_start) / a;
  if (interpolated_acceleration) {
    *interpolated_acceleration = a;
  }
  return time_interp;
}

double ConstantAccelerationInterpolationByTime(
    double time_start, double distance_along_path_start, double velocity_start,
    double time_end, double distance_along_path_end, double velocity_end,
    double time_interp, double* interpolated_acceleration) {
  assert(time_start <= time_interp && time_interp <= time_end);
  if (ApproxEqual(distance_along_path_start, distance_along_path_end) ||
      ApproxEqual(time_start, time_end)) {
    if (interpolated_acceleration) {
      *interpolated_acceleration = 0;
    }
    return distance_along_path_start;
  }
  double a = (velocity_end - velocity_start) / (time_end - time_start);
  double dt = time_interp - time_start;
  double distance_along_path_interp =
      distance_along_path_start + velocity_start * dt + 0.5 * a * dt * dt;
  if (interpolated_acceleration) {
    *interpolated_acceleration = a;
  }
  return distance_along_path_interp;
}

TrajectoryPoint InterpolateBetweenTrajectoryPointsByDistanceAlongPath(
    double distance_along_path_interp, const TrajectoryPoint& point_before,
    const TrajectoryPoint& point_after, double* interpolated_acceleration) {
  if (ApproxEqual(point_before.distance_along_path,
                  point_after.distance_along_path) ||
      ApproxEqual(point_before.time, point_after.time)) {
    return point_before;
  }
  assert(distance_along_path_interp <= point_after.distance_along_path &&
         distance_along_path_interp >= point_before.distance_along_path);
  const double distance_along_path_interp_fraction =
      (distance_along_path_interp - point_before.distance_along_path) /
      (point_after.distance_along_path - point_before.distance_along_path);
  TrajectoryPoint interpolated_pt;
  interpolated_pt.distance_along_path = distance_along_path_interp;
  interpolated_pt.x = LinearInterpolation(point_before.x, point_after.x,
                                          distance_along_path_interp_fraction);
  interpolated_pt.y = LinearInterpolation(point_before.y, point_after.y,
                                          distance_along_path_interp_fraction);
  interpolated_pt.yaw = LinearAngleInterpolation(
      point_before.yaw, point_after.yaw, distance_along_path_interp_fraction);
  interpolated_pt.curvature =
      LinearInterpolation(point_before.curvature, point_after.curvature,
                          distance_along_path_interp_fraction);
  interpolated_pt.time = ConstantAccelerationInterpolationByDistanceAlongPath(
      point_before.time, point_before.distance_along_path,
      point_before.velocity, point_after.time, point_after.distance_along_path,
      point_after.velocity, distance_along_path_interp,
      interpolated_acceleration);
  const double t_interp_fraction = (interpolated_pt.time - point_before.time) /
                                   (point_after.time - point_before.time);
  interpolated_pt.velocity = LinearInterpolation(
      point_before.velocity, point_after.velocity, t_interp_fraction);
  return interpolated_pt;
}

TrajectoryPoint InterpolateBetweenTrajectoryPointsByTime(
    double time_interp, const TrajectoryPoint& point_before,
    const TrajectoryPoint& point_after, double* interpolated_acceleration) {
  assert(time_interp <= point_after.time && time_interp >= point_before.time);
  if (ApproxEqual(point_before.time, point_after.time)) {
    return point_before;
  }
  double distance_along_path_interp = ConstantAccelerationInterpolationByTime(
      point_before.time, point_before.distance_along_path,
      point_before.velocity, point_after.time, point_after.distance_along_path,
      point_after.velocity, time_interp, interpolated_acceleration);
  TrajectoryPoint interpolated_pt;
  if (point_before.distance_along_path == point_after.distance_along_path) {
    // should only happen when stopped
    interpolated_pt = point_before;
    interpolated_pt.time = time_interp;
    return interpolated_pt;
  }
  const double distance_along_path_interp_fraction =
      (distance_along_path_interp - point_before.distance_along_path) /
      (point_after.distance_along_path - point_before.distance_along_path);
  const double t_interp_fraction = (time_interp - point_before.time) /
                                   (point_after.time - point_before.time);
  interpolated_pt.distance_along_path = distance_along_path_interp;
  interpolated_pt.x = LinearInterpolation(point_before.x, point_after.x,
                                          distance_along_path_interp_fraction);
  interpolated_pt.y = LinearInterpolation(point_before.y, point_after.y,
                                          distance_along_path_interp_fraction);
  interpolated_pt.yaw = LinearAngleInterpolation(
      point_before.yaw, point_after.yaw, distance_along_path_interp_fraction);
  interpolated_pt.curvature =
      LinearInterpolation(point_before.curvature, point_after.curvature,
                          distance_along_path_interp_fraction);
  interpolated_pt.velocity = LinearInterpolation(
      point_before.velocity, point_after.velocity, t_interp_fraction);
  interpolated_pt.time = time_interp;
  return interpolated_pt;
}

TrajectoryPoint InterpolateTrajectoryByDistanceAlongPath(
    double s_query, Trajectory::const_iterator trajectory_begin,
    Trajectory::const_iterator trajectory_end,
    double* interpolated_acceleration) {
  assert(trajectory_begin != trajectory_end);
  if (std::next(trajectory_begin) == trajectory_end) {
    return *trajectory_begin;
  }
  auto lower_bound_it = std::lower_bound(
      trajectory_begin, trajectory_end, s_query,
      [](const TrajectoryPoint& trajectory_point, const double s) -> bool {
        return trajectory_point.distance_along_path < s;
      });
  if (lower_bound_it == trajectory_end) {
    return *std::prev(trajectory_end);
  }
  if (lower_bound_it != trajectory_begin) {
    lower_bound_it = std::prev(lower_bound_it);
  }

  return InterpolateBetweenTrajectoryPointsByDistanceAlongPath(
      s_query, *lower_bound_it, *std::next(lower_bound_it),
      interpolated_acceleration);
}

TrajectoryPoint InterpolateTrajectoryByTime(
    double t_query, Trajectory::const_iterator trajectory_begin,
    Trajectory::const_iterator trajectory_end,
    double* interpolated_acceleration) {
  assert(trajectory_begin != trajectory_end);
  if (std::next(trajectory_begin) == trajectory_end) {
    return *trajectory_begin;
  }
  auto lower_bound_it = std::lower_bound(
      trajectory_begin, trajectory_end, t_query,
      [](const TrajectoryPoint& trajectory_point, const double t) -> bool {
        return trajectory_point.time < t;
      });
  if (lower_bound_it == trajectory_end) {
    return *std::prev(trajectory_end);
  }
  if (lower_bound_it != trajectory_begin) {
    lower_bound_it = std::prev(lower_bound_it);
  }

  return InterpolateBetweenTrajectoryPointsByTime(t_query, *lower_bound_it,
                                                  *std::next(lower_bound_it),
                                                  interpolated_acceleration);
}

TrajectoryPoint MatchBetweenTrajectoryPoints(Eigen::Vector2d query_point,
                                             const TrajectoryPoint& start_point,
                                             const TrajectoryPoint& end_point) {
  double interp_fraction;
  auto dist =
      SquaredDistanceToLineSegment(query_point, start_point.Position(),
                                   end_point.Position(), &interp_fraction);

  double distance_along_path_interp =
      start_point.distance_along_path +
      interp_fraction *
          (end_point.distance_along_path - start_point.distance_along_path);
  return InterpolateBetweenTrajectoryPointsByDistanceAlongPath(
      distance_along_path_interp, start_point, end_point);
}

TrajectoryPoint MatchToTrajectory(Eigen::Vector2d query_point,
                                  Trajectory::const_iterator trajectory_begin,
                                  Trajectory::const_iterator trajectory_end) {
  assert(trajectory_begin != trajectory_end &&
         std::next(trajectory_begin) != trajectory_end);
  auto bracketing_points =
      FindClosestSegment(query_point, trajectory_begin, trajectory_end);
  return MatchBetweenTrajectoryPoints(query_point, *bracketing_points.first,
                                      *bracketing_points.second);
}

Trajectory GenerateTrajectoryFromVelocityProfileAndPath(
    const Path& path, const Trajectory1D& velocity_profile,
    double time_resolution, double time_horizon) {
  assert(path.size() >= 2);
  assert(velocity_profile.size() >= 2);
  Trajectory trajectory;
  const double horizon = std::min(velocity_profile.front().time + time_horizon,
                                  velocity_profile.back().time);
  double current_time = velocity_profile.front().time;
  while (current_time <= horizon) {
    Trajectory1DPoint velocity_profile_point = InterpolateTrajectory1D(
        current_time, std::begin(velocity_profile), std::end(velocity_profile));
    if (path.front().distance_along_path <= velocity_profile_point.distance &&
        velocity_profile_point.distance <= path.back().distance_along_path) {
      PathPoint path_point = InterpolatePathByDistanceAlongPath(
          velocity_profile_point.distance, std::begin(path), std::end(path));
      TrajectoryPoint trajectory_point;
      trajectory_point.time = velocity_profile_point.time;
      trajectory_point.distance_along_path = path_point.distance_along_path;
      trajectory_point.x = path_point.x;
      trajectory_point.y = path_point.y;
      trajectory_point.yaw = path_point.yaw;
      trajectory_point.curvature = path_point.curvature;
      trajectory_point.velocity = velocity_profile_point.velocity;
      trajectory.push_back(trajectory_point);
    }
    current_time += time_resolution;
  }
  return trajectory;
}

}  // namespace bookbot

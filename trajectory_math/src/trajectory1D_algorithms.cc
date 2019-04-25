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
#include <trajectory_math/curve_algorithms.h>
#include <trajectory_math/float_comparison.h>
#include <trajectory_math/trajectory1D_algorithms.h>

namespace bookbot {

Trajectory1DPoint InterpolateBetweenTrajectory1DPoints(
    double time_interp, const Trajectory1DPoint& point_before,
    const Trajectory1DPoint& point_after) {
  assert(point_before.time <= time_interp && time_interp <= point_after.time);
  Trajectory1DPoint interpolated_point;
  if (ApproxEqual(point_after.time, point_before.time)) {
    return point_before;
  }
  interpolated_point.time = time_interp;
  interpolated_point.distance = CubicHermiteInterpolate(
      time_interp, point_before.time, point_before.distance,
      point_before.velocity, point_after.time, point_after.distance,
      point_after.velocity);
  interpolated_point.velocity = CubicHermiteInterpolateDerivative(
      time_interp, point_before.time, point_before.distance,
      point_before.velocity, point_after.time, point_after.distance,
      point_after.velocity);
  interpolated_point.acceleration = CubicHermiteInterpolateSecondDerivative(
      time_interp, point_before.time, point_before.distance,
      point_before.velocity, point_after.time, point_after.distance,
      point_after.velocity);
  return interpolated_point;
}

Trajectory1DPoint InterpolateTrajectory1D(
    double time_interp, Trajectory1D::const_iterator trajectory_begin,
    Trajectory1D::const_iterator trajectory_end) {
  assert(trajectory_begin != trajectory_end);
  if (std::next(trajectory_begin) == trajectory_end) {
    return *trajectory_begin;
  }
  auto lower_bound_it = std::lower_bound(
      trajectory_begin, trajectory_end, time_interp,
      [](const Trajectory1DPoint& trajectory_point, const double time) -> bool {
        return trajectory_point.time < time;
      });
  if (lower_bound_it == trajectory_end) {
    return *std::prev(trajectory_end);
  }
  if (lower_bound_it != trajectory_begin) {
    lower_bound_it = std::prev(lower_bound_it);
  }

  return InterpolateBetweenTrajectory1DPoints(time_interp, *lower_bound_it,
                                              *std::next(lower_bound_it));
}

double MinTimeToVelocity(double iniital_velocity, double target_velocity,
                         Interval acceleration_constraints,
                         double* minimum_distance) {
  const double delta_v = std::abs(iniital_velocity - target_velocity);
  const double acceleration_limit = (iniital_velocity < target_velocity)
                                        ? acceleration_constraints.max_value
                                        : -acceleration_constraints.min_value;
  assert(acceleration_limit > 0);
  const double min_time = delta_v / acceleration_limit;
  if (minimum_distance) {
    *minimum_distance =
        (iniital_velocity + 0.5 * acceleration_limit * min_time) * min_time;
  }
  return min_time;
}

}  // namespace bookbot

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

#ifndef BOOKBOT_TRAJECTORY_MATH_TRAJECTORY1D_ALGORITHMS_H_
#define BOOKBOT_TRAJECTORY_MATH_TRAJECTORY1D_ALGORITHMS_H_
#include <trajectory_math/curve_algorithms.h>
#include <trajectory_math/trajectory1D.h>

#include <vector>

namespace bookbot {

// Interpolation
Trajectory1DPoint InterpolateBetweenTrajectory1DPoints(
    double time_interp, const Trajectory1DPoint& point_before,
    const Trajectory1DPoint& point_after);

Trajectory1DPoint InterpolateTrajectory1D(
    double time_interp, Trajectory1D::const_iterator trajectory_begin,
    Trajectory1D::const_iterator trajectory_end);

// Dynamic limits
/// \brief Computes the minimum time and distance to reach a specified velocity
/// \note This calculation assumes no constraint on jerk (instantaneous
/// acceleration change)
/// \param [in] iniital_velocity          initial velocity
/// \param [in] target_velocity           target velocity
/// \param [in] acceleration constraints  min and max allowed acceleration
/// \param [out] min_distance             the minimum distance required
/// \return the minimum time required
double MinTimeToVelocity(double iniital_velocity, double target_velocity,
                         Interval acceleration_constraints,
                         double* min_distance = nullptr);

}  // namespace bookbot

#endif  // BOOKBOT_TRAJECTORY_MATH_TRAJECTORY1D_ALGORITHMS_H_

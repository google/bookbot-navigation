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

#ifndef BOOKBOT_PRIMITIVE_VELOCITY_PLANNER_VELOCITY_PROFILE_SEARCH_H_
#define BOOKBOT_PRIMITIVE_VELOCITY_PLANNER_VELOCITY_PROFILE_SEARCH_H_

#include <primitive_velocity_planner/velocity_planner_types.h>
#include <signed_distance_field/signed_distance_field.h>
#include <trajectory_math/path.h>
#include <trajectory_math/trajectory1D.h>

namespace bookbot {

Trajectory PerformVelocityProfileSearch(
    const Path& desired_path, const Trajectory1DPoint& initial_state,
    const VelocityPlannerParams& params, const Trajectory& previous_trajectory,
    const SignedDistanceField& distance_field);

}  // namespace bookbot

#endif  // BOOKBOT_PRIMITIVE_VELOCITY_PLANNER_VELOCITY_PROFILE_SEARCH_H_

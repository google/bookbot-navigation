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

#ifndef BOOKBOT_PRIMITIVE_VELOCITY_PLANNER_INTROSPECTION_H_
#define BOOKBOT_PRIMITIVE_VELOCITY_PLANNER_INTROSPECTION_H_

#include <trajectory_math/path.h>
#include <trajectory_math/trajectory.h>

#include <string>
#include <utility>
#include <vector>

namespace bookbot {

void PublishIntrospection(const std::string& topic, const std::string& frame,
                          const Trajectory& trajectory);

void PublishIntrospection(
    const std::string& topic, const std::string& frame,
    const std::vector<std::pair<TrajectoryPoint, TrajectoryPoint>>&
        expanded_tree);

void PublishIntrospection(const std::string& topic, const std::string& frame,
                          const Path& path);

}  // namespace bookbot

#endif  // BOOKBOT_PRIMITIVE_VELOCITY_PLANNER_INTROSPECTION_H_

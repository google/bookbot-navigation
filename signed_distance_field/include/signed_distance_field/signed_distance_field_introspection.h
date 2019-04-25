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

#ifndef BOOKBOT_SIGNED_DISTANCE_FIELD_INTROSPECTION_H_
#define BOOKBOT_SIGNED_DISTANCE_FIELD_INTROSPECTION_H_

#include <signed_distance_field/signed_distance_field.h>

namespace bookbot {

void PublishIntrospection(const std::string& topic, const std::string& frame,
                          const SignedDistanceField& distance_field,
                          double visualization_resolution,
                          double map_scaling_factor = 10);

void PublishConfigurationSpaceIntrospection(
    const std::string& topic, const std::string& frame,
    const SignedDistanceField& distance_field, double visualization_resolution,
    double robot_radius);

}  // namespace bookbot

#endif  // BOOKBOT_SIGNED_DISTANCE_FIELD_INTROSPECTION_H_

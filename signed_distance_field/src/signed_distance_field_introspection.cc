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

#include <nav_msgs/OccupancyGrid.h>
#include <ros_utilities/introspection.h>
#include <signed_distance_field/signed_distance_field_introspection.h>

namespace bookbot {

void PublishIntrospection(const std::string& topic, const std::string& frame,
                          const SignedDistanceField& distance_field,
                          double visualization_resolution,
                          double map_scaling_factor) {
  absl::optional<IntrospectionBackend>& backend = GetIntrospectionBackend();
  if (!backend.has_value()) {
    return;
  }

  nav_msgs::OccupancyGrid distance_field_msg;
  distance_field_msg.header.frame_id = frame;
  distance_field_msg.header.stamp = ros::Time::now();
  distance_field_msg.info.origin.orientation.x = 0;
  distance_field_msg.info.origin.orientation.y = 0;
  distance_field_msg.info.origin.orientation.z = 0;
  distance_field_msg.info.origin.orientation.w = 1;

  const Eigen::Vector2d origin = distance_field.GetOrigin();
  distance_field_msg.info.origin.position.x = origin[0];
  distance_field_msg.info.origin.position.y = origin[1];
  distance_field_msg.info.resolution = visualization_resolution;

  const double field_width =
      distance_field.GetFieldValues().cols() * distance_field.GetResolution();
  const double field_height =
      distance_field.GetFieldValues().rows() * distance_field.GetResolution();
  distance_field_msg.info.width = field_width / visualization_resolution;
  distance_field_msg.info.height = field_height / visualization_resolution;
  distance_field_msg.data.reserve(distance_field_msg.info.width *
                                  distance_field_msg.info.height);
  for (int row = 0; row < distance_field_msg.info.height; ++row) {
    for (int column = 0; column < distance_field_msg.info.width; ++column) {
      const double x_value = distance_field_msg.info.origin.position.x +
                             column * distance_field_msg.info.resolution;
      const double y_value = distance_field_msg.info.origin.position.y +
                             row * distance_field_msg.info.resolution;
      double cell_value = distance_field.Evaluate({x_value, y_value});
      distance_field_msg.data.push_back(cell_value * map_scaling_factor);
    }
  }

  backend.value().Publish(topic, std::move(distance_field_msg));
}

void PublishConfigurationSpaceIntrospection(
    const std::string& topic, const std::string& frame,
    const SignedDistanceField& distance_field, double visualization_resolution,
    double robot_radius) {
  absl::optional<IntrospectionBackend>& backend = GetIntrospectionBackend();
  if (!backend.has_value()) {
    return;
  }

  nav_msgs::OccupancyGrid configuration_space_msg;
  configuration_space_msg.header.frame_id = frame;
  configuration_space_msg.header.stamp = ros::Time::now();
  configuration_space_msg.info.origin.orientation.x = 0;
  configuration_space_msg.info.origin.orientation.y = 0;
  configuration_space_msg.info.origin.orientation.z = 0;
  configuration_space_msg.info.origin.orientation.w = 1;

  const Eigen::Vector2d origin = distance_field.GetOrigin();
  configuration_space_msg.info.origin.position.x = origin[0];
  configuration_space_msg.info.origin.position.y = origin[1];
  configuration_space_msg.info.resolution = visualization_resolution;

  const double field_width =
      distance_field.GetFieldValues().cols() * distance_field.GetResolution();
  const double field_height =
      distance_field.GetFieldValues().rows() * distance_field.GetResolution();
  configuration_space_msg.info.width = field_width / visualization_resolution;
  configuration_space_msg.info.height = field_height / visualization_resolution;
  configuration_space_msg.data.reserve(configuration_space_msg.info.width *
                                       configuration_space_msg.info.height);
  for (int row = 0; row < configuration_space_msg.info.height; ++row) {
    for (int column = 0; column < configuration_space_msg.info.width;
         ++column) {
      const double x_value = configuration_space_msg.info.origin.position.x +
                             column * configuration_space_msg.info.resolution;
      const double y_value = configuration_space_msg.info.origin.position.y +
                             row * configuration_space_msg.info.resolution;
      const double distance = distance_field.Evaluate({x_value, y_value});
      const double cell_value = distance < robot_radius ? 100 : 0;
      configuration_space_msg.data.push_back(cell_value);
    }
  }

  backend.value().Publish(topic, std::move(configuration_space_msg));
}

}  // namespace bookbot

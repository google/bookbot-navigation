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

#include <primitive_velocity_planner/velocity_planner_introspection.h>
#include <ros_utilities/introspection.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen3/Eigen/Geometry>

namespace bookbot {

void PublishIntrospection(const std::string& topic, const std::string& frame,
                          const Trajectory& trajectory) {
  absl::optional<IntrospectionBackend>& backend = GetIntrospectionBackend();
  if (!backend.has_value()) {
    return;
  }

  visualization_msgs::Marker trajectory_msg;
  trajectory_msg.header.frame_id = frame;
  trajectory_msg.header.stamp = ros::Time::now();
  trajectory_msg.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_msg.id = 47;
  trajectory_msg.scale.x = 0.1;
  trajectory_msg.pose.orientation.w = 1.0;
  for (const TrajectoryPoint& point : trajectory) {
    geometry_msgs::Point marker_point;
    marker_point.x = point.x;
    marker_point.y = point.y;
    marker_point.z = point.velocity;
    trajectory_msg.points.push_back(marker_point);
    std_msgs::ColorRGBA marker_color;
    marker_color.r = 0;
    marker_color.g = 0;
    marker_color.b = 1;
    marker_color.a = 1;
    trajectory_msg.colors.push_back(marker_color);
  }

  backend.value().Publish(topic, std::move(trajectory_msg));
}

void PublishIntrospection(
    const std::string& topic, const std::string& frame,
    const std::vector<std::pair<TrajectoryPoint, TrajectoryPoint>>&
        expanded_tree) {
  absl::optional<IntrospectionBackend>& backend = GetIntrospectionBackend();
  if (!backend.has_value()) {
    return;
  }

  visualization_msgs::Marker trajectory_bundle_msg;
  trajectory_bundle_msg.header.frame_id = frame;
  trajectory_bundle_msg.header.stamp = ros::Time::now();
  trajectory_bundle_msg.type = visualization_msgs::Marker::LINE_LIST;
  trajectory_bundle_msg.id = 75;
  trajectory_bundle_msg.scale.x = 0.01;
  trajectory_bundle_msg.pose.orientation.w = 1.0;
  for (const std::pair<TrajectoryPoint, TrajectoryPoint>& primitive_segment :
       expanded_tree) {
    const TrajectoryPoint& start_pt = primitive_segment.first;
    const TrajectoryPoint& end_pt = primitive_segment.second;
    geometry_msgs::Point start_marker_point;
    start_marker_point.x = start_pt.x;
    start_marker_point.y = start_pt.y;
    start_marker_point.z = start_pt.velocity;
    trajectory_bundle_msg.points.push_back(start_marker_point);
    geometry_msgs::Point end_marker_point;
    end_marker_point.x = end_pt.x;
    end_marker_point.y = end_pt.y;
    end_marker_point.z = end_pt.velocity;
    trajectory_bundle_msg.points.push_back(end_marker_point);
    std_msgs::ColorRGBA marker_color;
    marker_color.r = 0;
    marker_color.g = 1;
    marker_color.b = 1;
    marker_color.a = 0.7;
    trajectory_bundle_msg.colors.push_back(marker_color);
    trajectory_bundle_msg.colors.push_back(marker_color);
  }

  backend.value().Publish(topic, std::move(trajectory_bundle_msg));
}

void PublishIntrospection(const std::string& topic, const std::string& frame,
                          const Path& path) {
  absl::optional<IntrospectionBackend>& backend = GetIntrospectionBackend();
  if (!backend.has_value()) {
    return;
  }

  visualization_msgs::Marker path_msg;
  path_msg.header.frame_id = frame;
  path_msg.header.stamp = ros::Time::now();
  path_msg.type = visualization_msgs::Marker::LINE_STRIP;
  path_msg.id = 23;
  path_msg.scale.x = 0.1;
  path_msg.pose.orientation.w = 1.0;
  for (const PathPoint& point : path) {
    geometry_msgs::Point marker_point;
    marker_point.x = point.x;
    marker_point.y = point.y;
    marker_point.z = 0.;
    path_msg.points.push_back(marker_point);
    std_msgs::ColorRGBA marker_color;
    marker_color.r = 1;
    marker_color.g = 0.5;
    marker_color.b = 0;
    marker_color.a = 1;
    path_msg.colors.push_back(marker_color);
  }

  backend.value().Publish(topic, std::move(path_msg));
}

}  // namespace bookbot

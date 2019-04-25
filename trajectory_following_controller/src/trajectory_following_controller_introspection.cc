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

#include <trajectory_following_controller/trajectory_following_controller_introspection.h>

#include <ros_utilities/introspection.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace bookbot {

void PublishIntrospection(std::string topic, std::string frame,
                          Eigen::Vector2d robot_position, double robot_yaw,
                          const ControlIntrospection& introspection) {
  absl::optional<IntrospectionBackend>& backend = GetIntrospectionBackend();
  if (!backend.has_value()) {
    return;
  }

  ros::Time current_time = ros::Time::now();

  visualization_msgs::MarkerArray introspection_msg;
  {
    visualization_msgs::Marker point_introspection_msg;
    point_introspection_msg.header.frame_id = frame;
    point_introspection_msg.header.stamp = current_time;
    point_introspection_msg.type = visualization_msgs::Marker::SPHERE_LIST;
    point_introspection_msg.id = 1;
    point_introspection_msg.scale.x = 0.1;
    {
      geometry_msgs::Point marker_point;
      marker_point.x = introspection.matched_point.x;
      marker_point.y = introspection.matched_point.y;
      marker_point.z = 0;
      point_introspection_msg.points.push_back(marker_point);
      std_msgs::ColorRGBA marker_color;
      marker_color.r = 0;
      marker_color.g = 0;
      marker_color.b = 1;
      marker_color.a = 1;
      point_introspection_msg.colors.push_back(marker_color);
    }
    {
      geometry_msgs::Point marker_point;
      marker_point.x = introspection.lookahead_point[0];
      marker_point.y = introspection.lookahead_point[1];
      marker_point.z = 0;
      point_introspection_msg.points.push_back(marker_point);
      std_msgs::ColorRGBA marker_color;
      marker_color.r = 1;
      marker_color.g = 0;
      marker_color.b = 0;
      marker_color.a = 1;
      point_introspection_msg.colors.push_back(marker_color);
    }
    {
      geometry_msgs::Point marker_point;
      marker_point.x = introspection.spatially_matched_point.x;
      marker_point.y = introspection.spatially_matched_point.y;
      marker_point.z = 0;
      point_introspection_msg.points.push_back(marker_point);
      std_msgs::ColorRGBA marker_color;
      marker_color.r = 1;
      marker_color.g = 0;
      marker_color.b = 1;
      marker_color.a = 1;
      point_introspection_msg.colors.push_back(marker_color);
    }
    {
      geometry_msgs::Point marker_point;
      marker_point.x = robot_position[0];
      marker_point.y = robot_position[1];
      marker_point.z = 0;
      point_introspection_msg.points.push_back(marker_point);
      std_msgs::ColorRGBA marker_color;
      marker_color.r = 0;
      marker_color.g = 1;
      marker_color.b = 0;
      marker_color.a = 1;
      point_introspection_msg.colors.push_back(marker_color);
    }
    introspection_msg.markers.push_back(point_introspection_msg);
  }
  if (std::abs(introspection.desired_curvature) > 0.1) {
    visualization_msgs::Marker curvature_introspection_msg;
    curvature_introspection_msg.header.frame_id = frame;
    curvature_introspection_msg.header.stamp = ros::Time::now();
    curvature_introspection_msg.type = visualization_msgs::Marker::ARROW;
    curvature_introspection_msg.id = 2;
    curvature_introspection_msg.scale.x = introspection.desired_curvature;
    curvature_introspection_msg.scale.y = 0.1;
    curvature_introspection_msg.scale.z = 0.1;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(robot_yaw + M_PI / 2),
                          curvature_introspection_msg.pose.orientation);
    curvature_introspection_msg.pose.position.x = robot_position[0];
    curvature_introspection_msg.pose.position.y = robot_position[1];
    curvature_introspection_msg.pose.position.z = 0;
    std_msgs::ColorRGBA marker_color;
    marker_color.r = 1;
    marker_color.g = 0;
    marker_color.b = 0;
    marker_color.a = 1;
    curvature_introspection_msg.color = marker_color;
    introspection_msg.markers.push_back(curvature_introspection_msg);
  }

  backend.value().Publish(topic, std::move(introspection_msg));
}

}  // namespace bookbot

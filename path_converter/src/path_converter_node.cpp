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

#include <planning_msgs/Path.h>
#include <planning_msgs/WaypointList.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <trajectory_math/curve_algorithms.h>
#include <trajectory_math/point_algorithms.h>

#include <Eigen/Core>

constexpr char kInputTopic[] = "/waypoints";
constexpr char kOutputTopic[] = "/desired_path";
constexpr double kDesiredSpacing = 0.5;

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_converter");

  ros::NodeHandle node_handle;
  ros::Publisher publisher =
      node_handle.advertise<planning_msgs::Path>(kOutputTopic, 10);

  boost::function<void(const planning_msgs::WaypointList&)>
      subscriber_callback =
          [&publisher](
              const planning_msgs::WaypointList& waypoint_msg) -> void {
    planning_msgs::Path plan_msg;
    plan_msg.header.stamp = waypoint_msg.header.stamp;
    plan_msg.header.frame_id = waypoint_msg.header.frame_id;
    for (int i = 0; i < waypoint_msg.waypoints.size(); ++i) {
      planning_msgs::PathPoint point_msg;
      const auto& waypoint = waypoint_msg.waypoints[i];
      point_msg.x = waypoint.x;
      point_msg.y = waypoint.y;

      // Get s value based on distance from previous point
      point_msg.distance_along_path = 0.;
      if (!plan_msg.points.empty()) {
        const planning_msgs::PathPoint& prev_point_msg = plan_msg.points.back();
        const double dx = point_msg.x - prev_point_msg.x;
        const double dy = point_msg.y - prev_point_msg.y;
        const double distance = std::sqrt(dx * dx + dy * dy);

        // Linearly interpolate points at desired spacing around corners
        if (distance >= 2 * kDesiredSpacing) {
          planning_msgs::PathPoint interpolation_point_after_msg;
          const double interpolation_fraction = kDesiredSpacing / distance;
          interpolation_point_after_msg.distance_along_path =
              prev_point_msg.distance_along_path + kDesiredSpacing;
          interpolation_point_after_msg.x = bookbot::LinearInterpolation(
              prev_point_msg.x, point_msg.x, interpolation_fraction);
          interpolation_point_after_msg.y = bookbot::LinearInterpolation(
              prev_point_msg.y, point_msg.y, interpolation_fraction);
          plan_msg.points.push_back(interpolation_point_after_msg);

          planning_msgs::PathPoint interpolation_point_before_msg;
          interpolation_point_before_msg.distance_along_path =
              prev_point_msg.distance_along_path + distance - kDesiredSpacing;
          interpolation_point_before_msg.x = bookbot::LinearInterpolation(
              prev_point_msg.x, point_msg.x, 1 - interpolation_fraction);
          interpolation_point_before_msg.y = bookbot::LinearInterpolation(
              prev_point_msg.y, point_msg.y, 1 - interpolation_fraction);
          plan_msg.points.push_back(interpolation_point_before_msg);
        }

        point_msg.distance_along_path =
            prev_point_msg.distance_along_path + distance;
      }
      plan_msg.points.push_back(point_msg);
    }

    for (int i = 0; i < plan_msg.points.size(); ++i) {
      auto& current_point = plan_msg.points[i];

      // get yaw value based on sequential points
      current_point.yaw = 0.;
      if (i + 1 < plan_msg.points.size()) {
        const auto& next_point = plan_msg.points[i + 1];
        current_point.yaw = std::atan2(next_point.y - current_point.y,
                                       next_point.x - current_point.x);
      } else if (i > 0) {
        const auto& previous_point = plan_msg.points[i - 1];
        current_point.yaw = std::atan2(current_point.y - previous_point.y,
                                       current_point.x - previous_point.x);
      }

      // get kappa value based on three point curvature approximation
      current_point.curvature = 0.;
      if (i > 0 && i + 1 < plan_msg.points.size()) {
        const auto& previous_point = plan_msg.points[i - 1];
        const auto& next_point = plan_msg.points[i + 1];
        current_point.curvature = bookbot::ApproximateCurvatureMagnitude(
            {previous_point.x, previous_point.y},
            {current_point.x, current_point.y}, {next_point.x, next_point.y});
      }
    }
    publisher.publish(plan_msg);
  };

  ros::Subscriber subscriber =
      node_handle.subscribe<planning_msgs::WaypointList>(kInputTopic, 10,
                                                         subscriber_callback);

  ros::spin();
}

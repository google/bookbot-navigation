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

#ifndef BOOKBOT_WAYPOINT_BACKEND_WAYPOINT_BACKEND_H_
#define BOOKBOT_WAYPOINT_BACKEND_WAYPOINT_BACKEND_H_

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros_utilities/topic_listener.h>
#include <std_msgs/Empty.h>

#include <Eigen/Core>
#include <mutex>

constexpr char kWaypointAddTopic[] = "/waypoint/add";
constexpr char kWaypointDeleteTopic[] = "/waypoint/delete";
constexpr char kWaypointResetTopic[] = "/waypoint/reset";
constexpr char kWaypointStopTopic[] = "/waypoint/stop";
constexpr char kWaypointOutputTopic[] = "/waypoints";
constexpr char kOdomTopic[] = "/odom";
constexpr char kOdomFrame[] = "/odom";
constexpr double kDeleteRadius = 0.5;

namespace bookbot {

class WaypointBackend {
 public:
  WaypointBackend(const ros::NodeHandle& node_handle);

  void AddWaypointInRobotFrame(
      const Eigen::Vector3d& point_to_add_in_robot_frame,
      ros::Time current_time);
  void AddWaypoint(const Eigen::Vector3d& point_to_add_in_odom_frame);
  void DeleteWaypointInRobotFrame(
      const Eigen::Vector3d& point_to_delete_in_robot_frame,
      ros::Time current_time);
  void DeleteWaypoint(const Eigen::Vector3d& point_to_delete_in_odom_frame);
  void ClearWaypoints();
  void PublishWaypoints();

  // Callbacks
  void WaypointAddCallback(const geometry_msgs::Point& point_to_add_msg);
  void WaypointDeleteCallback(const geometry_msgs::Point& point_to_delete_msg);
  void WaypointResetCallback(const geometry_msgs::Point& point_to_add_msg);
  void WaypointStopCallback(const std_msgs::Empty& point_to_add_msg);

 private:
  // Protects all members for ensuring sequential (ADD/DELETE/etc) operations
  std::mutex waypoint_backend_mutex_;

  // Protects ordered_waypoints within a single (ADD/DELETE/etc) operation
  std::mutex ordered_waypoints_mutex_;

  std::vector<Eigen::Vector3d> ordered_waypoints_;  // Stored in odom frame

  ros::NodeHandle node_handle_;
  ros::Subscriber waypoint_add_subscriber_;
  ros::Subscriber waypoint_delete_subscriber_;
  ros::Subscriber waypoint_reset_subscriber_;
  ros::Subscriber waypoint_stop_subscriber_;
  ros::Publisher waypoints_publisher_;
  TopicListener<nav_msgs::Odometry> odometry_listener_;
};

}  // namespace bookbot

#endif  // BOOKBOT_WAYPOINT_BACKEND_WAYPOINT_BACKEND_H_

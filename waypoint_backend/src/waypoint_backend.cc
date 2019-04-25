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

#include <eigen_conversions/eigen_msg.h>
#include <planning_msgs/WaypointList.h>
#include <waypoint_backend/waypoint_backend.h>

#include <algorithm>

constexpr double kPrefixLength = 0.3;
constexpr double kMinElementLengthForCurvatureEstimation = 1.0;

namespace bookbot {

WaypointBackend::WaypointBackend(const ros::NodeHandle& node_handle)
    : odometry_listener_(kOdomTopic, node_handle), node_handle_(node_handle) {
  waypoint_add_subscriber_ = node_handle_.subscribe(
      kWaypointAddTopic, 10, &WaypointBackend::WaypointAddCallback, this);
  waypoint_delete_subscriber_ = node_handle_.subscribe(
      kWaypointDeleteTopic, 10, &WaypointBackend::WaypointDeleteCallback, this);
  waypoint_reset_subscriber_ = node_handle_.subscribe(
      kWaypointResetTopic, 10, &WaypointBackend::WaypointResetCallback, this);
  waypoint_stop_subscriber_ = node_handle_.subscribe(
      kWaypointStopTopic, 10, &WaypointBackend::WaypointStopCallback, this);

  waypoints_publisher_ = node_handle_.advertise<planning_msgs::WaypointList>(
      kWaypointOutputTopic, 1);
}

void WaypointBackend::AddWaypointInRobotFrame(
    const Eigen::Vector3d& point_to_add_in_robot_frame,
    ros::Time current_time) {
  // Get latest robot_T_odom transformation
  nav_msgs::Odometry latest_odom_msg;
  if (!odometry_listener_.GetClosestBefore(current_time, &latest_odom_msg)) {
    ROS_ERROR("Can't add waypoint because odometry queue is empty");
    return;
  }
  Eigen::Isometry3d odom_T_robot;
  tf::poseMsgToEigen(latest_odom_msg.pose.pose, odom_T_robot);

  Eigen::Vector3d point_to_add_odom =
      odom_T_robot * point_to_add_in_robot_frame;

  AddWaypoint(point_to_add_odom);
}

void WaypointBackend::AddWaypoint(
    const Eigen::Vector3d& point_to_add_in_odom_frame) {
  std::lock_guard<std::mutex> lock(ordered_waypoints_mutex_);
  ordered_waypoints_.push_back(point_to_add_in_odom_frame);
}

void WaypointBackend::DeleteWaypointInRobotFrame(
    const Eigen::Vector3d& point_to_delete_in_robot_frame,
    ros::Time current_time) {
  // Get latest robot_T_odom transformation
  nav_msgs::Odometry latest_odom_msg;
  if (!odometry_listener_.GetClosestBefore(current_time, &latest_odom_msg)) {
    ROS_ERROR("Can't add waypoint because odometry queue is empty");
    return;
  }
  Eigen::Isometry3d robot_T_odom;
  tf::poseMsgToEigen(latest_odom_msg.pose.pose, robot_T_odom);

  Eigen::Vector3d point_to_delete_odom =
      robot_T_odom * point_to_delete_in_robot_frame;

  DeleteWaypoint(point_to_delete_odom);
}

void WaypointBackend::DeleteWaypoint(
    const Eigen::Vector3d& point_to_delete_in_odom_frame) {
  std::lock_guard<std::mutex> lock(ordered_waypoints_mutex_);
  // Find closest waypoint to delete point
  auto closest_waypoint_iter = std::min_element(
      std::begin(ordered_waypoints_), std::end(ordered_waypoints_),
      [&point_to_delete_in_odom_frame](Eigen::Vector3d waypoint_a,
                                       Eigen::Vector3d waypoint_b) -> bool {
        const double squared_distance_a =
            Eigen::Vector3d(waypoint_a - point_to_delete_in_odom_frame)
                .squaredNorm();
        const double squared_distance_b =
            Eigen::Vector3d(waypoint_b - point_to_delete_in_odom_frame)
                .squaredNorm();
        return squared_distance_a < squared_distance_b;
      });
  if (Eigen::Vector3d(*closest_waypoint_iter - point_to_delete_in_odom_frame)
          .squaredNorm() < kDeleteRadius * kDeleteRadius) {
    ordered_waypoints_.erase(closest_waypoint_iter);
  }
}

void WaypointBackend::ClearWaypoints() {
  std::lock_guard<std::mutex> lock(ordered_waypoints_mutex_);
  ordered_waypoints_.clear();
}

void WaypointBackend::PublishWaypoints() {
  planning_msgs::WaypointList waypoints_list_msg;
  const auto publish_time = ros::Time::now();
  waypoints_list_msg.header.stamp = publish_time;
  waypoints_list_msg.header.frame_id = kOdomFrame;

  std::lock_guard<std::mutex> backend_lock(waypoint_backend_mutex_);
  std::lock_guard<std::mutex> waypoints_lock(ordered_waypoints_mutex_);
  if (ordered_waypoints_.size() > 1) {
    for (const Eigen::Vector3d& waypoint : ordered_waypoints_) {
      geometry_msgs::Point point_msg;
      point_msg.x = waypoint[0];
      point_msg.y = waypoint[1];
      waypoints_list_msg.waypoints.push_back(point_msg);
    }
  }

  waypoints_publisher_.publish(waypoints_list_msg);
}

// Callbacks
void WaypointBackend::WaypointAddCallback(
    const geometry_msgs::Point& point_to_add_msg) {
  const ros::Time current_time = ros::Time::now();
  Eigen::Vector3d point_to_add_odom;
  tf::pointMsgToEigen(point_to_add_msg, point_to_add_odom);

  std::lock_guard<std::mutex> lock(waypoint_backend_mutex_);
  if (ordered_waypoints_.empty()) {
    // Start path slightly behind the robot such that there is a prefix segment
    // of suitable length for curvature estimation, but that the distance
    // between the robot position and the end of the prefix segment is
    // kPrefixLength.
    AddWaypointInRobotFrame(
        Eigen::Vector3d(kPrefixLength - kMinElementLengthForCurvatureEstimation,
                        0, 0),
        current_time);
    AddWaypointInRobotFrame(Eigen::Vector3d(kPrefixLength, 0, 0), current_time);
  }
  AddWaypoint(point_to_add_odom);
}

void WaypointBackend::WaypointDeleteCallback(
    const geometry_msgs::Point& point_to_delete_msg) {
  const ros::Time current_time = ros::Time::now();
  Eigen::Vector3d point_to_delete_odom;
  tf::pointMsgToEigen(point_to_delete_msg, point_to_delete_odom);

  std::lock_guard<std::mutex> lock(waypoint_backend_mutex_);
  DeleteWaypoint(point_to_delete_odom);
}

void WaypointBackend::WaypointResetCallback(
    const geometry_msgs::Point& point_to_reset_msg) {
  const ros::Time current_time = ros::Time::now();
  Eigen::Vector3d point_to_reset_odom;
  tf::pointMsgToEigen(point_to_reset_msg, point_to_reset_odom);

  std::lock_guard<std::mutex> lock(waypoint_backend_mutex_);
  ClearWaypoints();
  // Start path slightly behind the robot such that there is a prefix segment
  // of suitable length for curvature estimation, but that the distance
  // between the robot position and the end of the prefix segment is
  // kPrefixLength.
  AddWaypointInRobotFrame(
      Eigen::Vector3d(kPrefixLength - kMinElementLengthForCurvatureEstimation,
                      0, 0),
      current_time);
  AddWaypointInRobotFrame(Eigen::Vector3d(kPrefixLength, 0, 0), current_time);
  AddWaypoint(point_to_reset_odom);
}

void WaypointBackend::WaypointStopCallback(
    const std_msgs::Empty& point_to_add_msg) {
  ROS_DEBUG("STOP");
  std::lock_guard<std::mutex> lock(waypoint_backend_mutex_);
  ClearWaypoints();
}

}  // namespace bookbot

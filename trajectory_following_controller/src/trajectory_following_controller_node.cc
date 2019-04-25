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

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <planning_msgs/Trajectory.h>
#include <ros/ros.h>
#include <ros_utilities/introspection.h>
#include <ros_utilities/topic_listener.h>
#include <tf/tf.h>
#include <trajectory_following_controller/trajectory_following_controller.h>
#include <trajectory_following_controller/trajectory_following_controller_introspection.h>
#include <trajectory_math/trajectory.h>
#include <visualization_msgs/MarkerArray.h>

#include <mutex>

constexpr char kCommandTopic[] = "/cmd_vel";
constexpr char kIntrospectionTopic[] =
    "/trajectory_following_controller/introspection";
constexpr char kOdomFrame[] = "/odom";
constexpr char kOdomTopic[] = "/odom";
constexpr char kTrajectoryTopic[] = "/trajectory";
constexpr int kCycleFrequency = 50;
constexpr double kTrajectoryTimeout = 0.5;

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_following_controller");

  ros::NodeHandle node_handle;
  bookbot::Introspector introspector(node_handle);
  std::mutex trajectory_mutex;
  bookbot::Trajectory last_recieved_trajectory;
  ros::Time last_recieved_trajectory_time;

  boost::function<void(const planning_msgs::Trajectory&)> trajectory_callback =
      [&trajectory_mutex, &last_recieved_trajectory,
       &last_recieved_trajectory_time](
          const planning_msgs::Trajectory& msg) -> void {
    bookbot::Trajectory new_trajectory;
    for (const planning_msgs::TrajectoryPoint& trajectory_point_msg :
         msg.points) {
      new_trajectory.push_back(bookbot::TrajectoryPoint(
          trajectory_point_msg.time, trajectory_point_msg.distance_along_path,
          trajectory_point_msg.x, trajectory_point_msg.y,
          trajectory_point_msg.yaw, 0., trajectory_point_msg.velocity));
    }
    std::lock_guard<std::mutex> lock(trajectory_mutex);
    std::swap(last_recieved_trajectory, new_trajectory);
    last_recieved_trajectory_time = msg.header.stamp;
  };
  ros::Subscriber subscriber = node_handle.subscribe<planning_msgs::Trajectory>(
      kTrajectoryTopic, 10, trajectory_callback);

  ros::Publisher control_command_publisher =
      node_handle.advertise<geometry_msgs::Twist>(kCommandTopic, 1);

  bookbot::TopicListener<nav_msgs::Odometry> odometry_listener(kOdomTopic,
                                                               node_handle);

  ros::Rate loop_rate(kCycleFrequency);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();

    geometry_msgs::Twist stop_command;
    stop_command.linear.x = 0;
    stop_command.angular.z = 0;

    const ros::Time current_time = ros::Time::now();

    {
      std::lock_guard<std::mutex> lock(trajectory_mutex);
      if (last_recieved_trajectory.empty()) {
        control_command_publisher.publish(stop_command);
        continue;
      }
      if (ros::Duration(current_time - last_recieved_trajectory_time).toSec() >
          kTrajectoryTimeout) {
        ROS_DEBUG_STREAM("Trajectory is too old");
        control_command_publisher.publish(stop_command);
        continue;
      }
    }

    // Get odom
    nav_msgs::Odometry odom;
    if (!odometry_listener.GetClosestBefore(current_time, &odom)) {
      ROS_ERROR("Odometry queue is empty");
      control_command_publisher.publish(stop_command);
      continue;
    }
    const Eigen::Vector2d robot_position = {odom.pose.pose.position.x,
                                            odom.pose.pose.position.y};

    double robot_yaw = tf::getYaw(odom.pose.pose.orientation);

    // Compute control command
    bookbot::ControlCommand command;
    bookbot::ControlIntrospection introspection;
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex);
      command = bookbot::ComputeControlCommand(
          robot_position, robot_yaw, last_recieved_trajectory,
          current_time.toSec(), &introspection);
    }

    // Publish control command
    geometry_msgs::Twist command_msg;
    command_msg.linear.x = command.forward_velocity;
    command_msg.angular.z = command.angular_velocity;
    control_command_publisher.publish(command_msg);

    // Publish introspection
    bookbot::PublishIntrospection(kIntrospectionTopic, kOdomFrame,
                                  robot_position, robot_yaw, introspection);
  }
}

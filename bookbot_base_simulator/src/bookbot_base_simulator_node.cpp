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

#include <bookbot_base_simulator/bookbot_base_simulator.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <mutex>

constexpr char kCommandTopic[] = "/cmd_vel";
constexpr char kSimOdomTopic[] = "/odom";
constexpr char kSimOdomFrame[] = "/odom";
constexpr char kRobotFrame[] = "/base_link";
constexpr int kCycleFrequency = 50;

int main(int argc, char** argv) {
  ros::init(argc, argv, "base_simulator");

  ros::NodeHandle node_handle;
  std::mutex simulator_mutex;
  tf::TransformBroadcaster tf_broadcaster;

  bookbot::simulator::BaseSimulator simulator;
  ros::Time current_time = ros::Time::now();
  simulator.Initialize(bookbot::simulator::RobotSimulatorState(),
                       bookbot::simulator::SimTime(current_time.toSec()));

  boost::function<void(const geometry_msgs::Twist&)> command_callback =
      [&simulator, &simulator_mutex](const geometry_msgs::Twist& msg) -> void {
    bookbot::simulator::ControlCommand command;
    command.forward_velocity = msg.linear.x;
    command.angular_velocity = msg.angular.z;

    if (std::isfinite(command.forward_velocity) &&
        std::isfinite(command.angular_velocity)) {
      std::lock_guard<std::mutex> lock(simulator_mutex);
      simulator.SetControlCommand(command);
    } else {
      ROS_ERROR_STREAM("received invalid command:" << msg.linear.x << " "
                                                   << msg.angular.z);
    }
  };

  ros::Subscriber command_subscriber =
      node_handle.subscribe<geometry_msgs::Twist>(kCommandTopic, 10,
                                                  command_callback);

  ros::Publisher odom_publisher =
      node_handle.advertise<nav_msgs::Odometry>(kSimOdomTopic, 10);

  ros::Rate loop_rate(kCycleFrequency);
  while (ros::ok()) {
    const ros::Time updated_time = ros::Time::now();
    const double delta_time = updated_time.toSec() - current_time.toSec();
    current_time = updated_time;
    bookbot::simulator::SimulatorWorldState sim_state;
    {
      std::lock_guard<std::mutex> lock(simulator_mutex);
      simulator.Step(delta_time);
      sim_state = simulator.GetCurrentSimState();
    }

    Eigen::Isometry3d sim_state_transform =
        bookbot::simulator::ExtractTransform(sim_state.robot_state);
    ros::Time sim_time = ros::Time(sim_state.sim_time.GetTimeAsDouble());

    // broadcast robot base motion as an odometry tf update
    tf::Transform sim_state_tf_transform;
    tf::transformEigenToTF(sim_state_transform, sim_state_tf_transform);
    tf_broadcaster.sendTransform(tf::StampedTransform(
        sim_state_tf_transform, sim_time, kSimOdomFrame, kRobotFrame));

    // publish odometry message
    nav_msgs::Odometry odom_message;
    odom_message.header.frame_id = kSimOdomFrame;
    odom_message.header.stamp = updated_time;
    odom_message.child_frame_id = kRobotFrame;
    tf::poseTFToMsg(sim_state_tf_transform, odom_message.pose.pose);
    odom_message.twist.twist.angular.z = sim_state.robot_state.angular_velocity;
    odom_message.twist.twist.linear.x = sim_state.robot_state.forward_velocity *
                                        std::cos(sim_state.robot_state.yaw);
    odom_message.twist.twist.linear.y = sim_state.robot_state.forward_velocity *
                                        std::sin(sim_state.robot_state.yaw);
    odom_publisher.publish(odom_message);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

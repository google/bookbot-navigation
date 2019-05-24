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
#include <nav_msgs/Odometry.h>
#include <planning_msgs/Path.h>
#include <planning_msgs/Trajectory.h>
#include <primitive_velocity_planner/primitive_velocity_planner.h>
#include <primitive_velocity_planner/velocity_planner_introspection.h>
#include <ros/ros.h>
#include <ros_utilities/introspection.h>
#include <ros_utilities/parameter_loading.h>
#include <ros_utilities/scoped_timer.h>
#include <ros_utilities/topic_listener.h>
#include <signed_distance_field/signed_distance_field.h>
#include <std_msgs/Empty.h>
#include <tf/tf.h>
#include <trajectory_math/trajectory.h>
#include <visualization_msgs/MarkerArray.h>

#include <iomanip>
#include <mutex>

constexpr char kPathTopic[] = "/desired_path";
constexpr char kOverrideTopic[] = "/obstacle_override";
constexpr char kPerceptionTopic[] = "/occupancy_grid";
constexpr char kOdomFrame[] = "/odom";
constexpr char kTrajectoryTopic[] = "/trajectory";
constexpr char kOdomTopic[] = "/odom";
constexpr char kIntrospectionTopic[] = "/introspection/velocity_planner";

int main(int argc, char** argv) {
  ros::init(argc, argv, "velocity_planner");

  ros::NodeHandle node_handle("velocity_planner");
  bookbot::Introspector introspector(node_handle);

  bookbot::Trajectory last_trajectory;
  bookbot::VelocityPlannerParams params;

  // Load required parameters
  if (!bookbot::ReadParameters(
          node_handle, "min_acceleration",
          &params.acceleration_bounds.constraint_interval.min_value,
          "max_acceleration",
          &params.acceleration_bounds.constraint_interval.max_value,
          "min_lateral_acceleration",
          &params.lateral_acceleration_bounds.constraint_interval.min_value,
          "max_lateral_acceleration",
          &params.lateral_acceleration_bounds.constraint_interval.max_value,
          "min_velocity", &params.velocity_bounds.constraint_interval.min_value,
          "max_velocity", &params.velocity_bounds.constraint_interval.max_value,
          "min_desired_velocity",
          &params.desired_velocity_bounds.constraint_interval.min_value,
          "max_desired_velocity",
          &params.desired_velocity_bounds.constraint_interval.max_value,
          "robot_radius", &params.robot_radius, "obstacle_check_front_offset",
          &params.obstacle_check_front_offset, "planning_time_horizon",
          &params.planning_time_horizon, "time_bins", &params.time_bins,
          "velocity_bins", &params.velocity_bins, "cycle_time",
          &params.velocity_planner_cycle_time, "reinitialization_distance",
          &params.reinitialization_distance, "max_occupancy_grid_value",
          &params.max_occupancy_grid_value,
          "max_yaw_change_for_nonzero_velocity",
          &params.max_yaw_change_for_nonzero_velocity,
          "visited_grid_distance_resolution",
          &params.visited_grid_distance_resolution, "path_end_buffer_distance",
          &params.path_end_buffer_distance, "travel_time_weight",
          &params.travel_time_weight, "jerk_weight", &params.jerk_weight,
          "longitudinal_acceleration_weight",
          &params.longitudinal_acceleration_weight,
          "lateral_acceleration_weight", &params.lateral_acceleration_weight,
          "obstacle_weight", &params.obstacle_weight, "desired_velocity_weight",
          &params.desired_velocity_weight, "dead_zone_distance",
          &params.dead_zone_distance, "dead_zone_velocity",
          &params.dead_zone_velocity, "dead_zone_deceleration",
          &params.dead_zone_deceleration, "require_perception",
          &params.require_perception, "desired_path_timeout",
          &params.desired_path_timeout, "odometry_timeout",
          &params.odometry_timeout, "perception_timeout",
          &params.perception_timeout, "obstacle_override_timeout",
          &params.obstacle_override_timeout, "obstacle_override_max_velocity",
          &params.obstacle_override_max_velocity,
          "obstacle_override_desired_velocity",
          &params.obstacle_override_desired_velocity)) {
    return EXIT_FAILURE;
  }
  params.odometry_frame_id = kOdomFrame;
  params.override_obstacles = false;

  std::mutex path_mutex;
  bookbot::Path last_recieved_path;
  ros::Time last_recieved_path_time;
  boost::function<void(const planning_msgs::Path&)> path_callback =
      [&path_mutex, &last_recieved_path,
       &last_recieved_path_time](const planning_msgs::Path& msg) -> void {
    bookbot::Path new_path;
    for (const planning_msgs::PathPoint& path_point_msg : msg.points) {
      new_path.push_back({path_point_msg.distance_along_path, path_point_msg.x,
                          path_point_msg.y, path_point_msg.yaw,
                          path_point_msg.curvature});
    }
    std::lock_guard<std::mutex> lock(path_mutex);
    std::swap(last_recieved_path, new_path);
    last_recieved_path_time = msg.header.stamp;
  };
  ros::Subscriber path_subscriber =
      node_handle.subscribe<planning_msgs::Path>(kPathTopic, 10, path_callback);

  std::mutex override_mutex;
  ros::Time last_override_timestamp = ros::Time(0);
  boost::function<void(const ros::MessageEvent<std_msgs::Empty>& message_event)>
      override_callback =
          [&override_mutex, &last_override_timestamp](
              const ros::MessageEvent<std_msgs::Empty>& message_event) -> void {
    std::lock_guard<std::mutex> lock(override_mutex);
    last_override_timestamp = message_event.getReceiptTime();
  };
  ros::Subscriber override_subscriber = node_handle.subscribe<std_msgs::Empty>(
      kOverrideTopic, 1, override_callback);

  ros::Publisher trajectory_publisher =
      node_handle.advertise<planning_msgs::Trajectory>(kTrajectoryTopic, 1);

  bookbot::TopicListener<nav_msgs::OccupancyGrid> grid_listener(
      kPerceptionTopic, node_handle, 3);
  bookbot::TopicListener<nav_msgs::Odometry> odometry_listener(
      kOdomTopic, node_handle, 500);

  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::Rate loop_rate(1 / params.velocity_planner_cycle_time);
  ros::Time frame_time = ros::Time::now();
  while (ros::ok()) {
    frame_time = ros::Time::now();  // Should be perception time
    loop_rate.reset();  // Publish next trajectory 1 cycle time from now

    bool valid_path;
    bookbot::Path local_path_copy;
    {
      std::lock_guard<std::mutex> lock(path_mutex);
      valid_path = !last_recieved_path.empty() &&
                   ros::Duration(frame_time - last_recieved_path_time).toSec() <
                       params.desired_path_timeout;
      local_path_copy = last_recieved_path;
    }
    if (!valid_path) {
      loop_rate.sleep();
      continue;
    }

    bool valid_override = false;
    {
      std::lock_guard<std::mutex> lock(override_mutex);
      valid_override =
          ros::Duration(ros::Time::now() - last_override_timestamp).toSec() <
          params.obstacle_override_timeout;
    }
    bookbot::VelocityPlannerParams updated_params = params;
    if (valid_override) {
      updated_params.desired_velocity_bounds.constraint_interval.max_value =
          params.obstacle_override_desired_velocity;
      updated_params.velocity_bounds.constraint_interval.max_value =
          params.obstacle_override_max_velocity;
      updated_params.override_obstacles = true;
    }

    bookbot::TimingAggregator timing_aggregator("VelocityPlannerTiming", true);

    // Get latest odometry and robot state
    nav_msgs::Odometry odom;
    if (!odometry_listener.GetClosestBefore(frame_time, &odom) ||
        ros::Duration(frame_time - odom.header.stamp).toSec() >
            params.odometry_timeout) {
      ROS_ERROR("Invalid or stale odometry");
      loop_rate.sleep();
      continue;
    }
    bookbot::RobotPlanningState robot_state;
    robot_state.time = frame_time.toSec();
    robot_state.x = odom.pose.pose.position.x;
    robot_state.y = odom.pose.pose.position.y;
    robot_state.yaw = tf::getYaw(odom.pose.pose.orientation);
    robot_state.velocity =
        Eigen::Vector2d(odom.twist.twist.linear.x, odom.twist.twist.linear.y)
            .norm();
    robot_state.angular_velocity = odom.twist.twist.angular.z;

    // Get latest signed distance field
    nav_msgs::OccupancyGrid grid;
    if (params.require_perception &&
        (!grid_listener.GetClosestBefore(frame_time, &grid) ||
         ros::Duration(frame_time - grid.header.stamp).toSec() >
             params.perception_timeout)) {
      ROS_ERROR("Invalid or stale perception");
      loop_rate.sleep();
      continue;
    }
    bookbot::SignedDistanceField distance_field;
    if (params.require_perception) {
      bookbot::ScopedTimer timer("GenerateSignedDistanceField");
      distance_field = bookbot::GenerateSignedDistanceField(
          grid, 1, params.max_occupancy_grid_value);
    }

    // Compute best trajectory
    bookbot::Trajectory best_trajectory = bookbot::PlanVelocityProfile(
        local_path_copy, robot_state, updated_params, last_trajectory,
        distance_field);

    last_trajectory = best_trajectory;

    // Publish trajectory
    planning_msgs::Trajectory trajectory_msg;
    trajectory_msg.header.stamp = frame_time;
    trajectory_msg.header.frame_id = kOdomFrame;
    for (const bookbot::TrajectoryPoint& point : best_trajectory) {
      planning_msgs::TrajectoryPoint point_msg;
      point_msg.time = point.time;
      point_msg.distance_along_path = point.distance_along_path;
      point_msg.x = point.x;
      point_msg.y = point.y;
      point_msg.yaw = point.yaw;
      point_msg.velocity = point.velocity;
      trajectory_msg.points.push_back(point_msg);
    }
    loop_rate.sleep();
    trajectory_publisher.publish(trajectory_msg);
  }
  return EXIT_SUCCESS;
}

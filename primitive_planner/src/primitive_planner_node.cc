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
#include <primitive_planner/primitive_planner.h>
#include <primitive_planner/primitive_planner_introspection.h>
#include <ros/ros.h>
#include <ros_utilities/introspection.h>
#include <ros_utilities/parameter_loading.h>
#include <ros_utilities/scoped_timer.h>
#include <ros_utilities/topic_listener.h>
#include <signed_distance_field/signed_distance_field.h>
#include <signed_distance_field/signed_distance_field_introspection.h>
#include <tf/transform_datatypes.h>
#include <trajectory_math/curve_algorithms.h>
#include <trajectory_math/path.h>
#include <trajectory_math/path_algorithms.h>
#include <trajectory_math/trajectory_algorithms.h>

#include <mutex>

constexpr char kClickedPathTopic[] = "/desired_path";
constexpr char kIntrospectionTopic[] = "/introspection/velocity_planner";
constexpr char kOdomFrame[] = "/odom";
constexpr char kOdomTopic[] = "/odom";
constexpr char kPerceptionTopic[] = "/occupancy_grid";
constexpr char kPlannedPathTopic[] = "/planned_path";
constexpr char kTrajectoryTopic[] = "/trajectory";

int main(int argc, char** argv) {
  ros::init(argc, argv, "primitive_planner");

  ros::NodeHandle node_handle("primitive_planner");
  bookbot::Introspector introspector(node_handle);

  ros::Publisher path_publisher =
      node_handle.advertise<planning_msgs::Path>(kPlannedPathTopic, 1);

  planning_msgs::Path last_planned_path_msg;

  bookbot::Path last_planned_path;
  bookbot::PrimitivePlanningParams params;

  // Load required parameters
  if (!bookbot::ReadParameters(
          node_handle, "robot_radius", &params.robot_radius,
          "obstacle_check_front_offset", &params.obstacle_check_front_offset,
          "primitive_min_turn_radius", &params.primitive_min_turn_radius,
          "num_primitive_curvature_samples",
          &params.num_primitive_curvature_samples,
          "max_curvature_change_per_meter",
          &params.max_curvature_change_per_meter, "horizon", &params.horizon,
          "travel_cost_weight", &params.travel_cost_weight,
          "path_divergence_weight", &params.path_divergence_weight,
          "path_end_divergence_weight", &params.path_end_divergence_weight,
          "path_end_alignment_weight", &params.path_end_alignment_weight,
          "max_path_divergence", &params.max_path_divergence, "obstacle_weight",
          &params.obstacle_weight, "curvature_integral_weight",
          &params.curvature_integral_weight, "primitive_length",
          &params.primitive_length, "visited_grid_spatial_resolution",
          &params.visited_grid_spatial_resolution,
          "num_visited_grid_angle_bins", &params.num_visited_grid_angle_bins,
          "visited_grid_curvature_resolution",
          &params.visited_grid_curvature_resolution, "require_perception",
          &params.require_perception, "clicked_path_timeout",
          &params.clicked_path_timeout, "odometry_timeout",
          &params.odometry_timeout, "perception_timeout",
          &params.perception_timeout, "trajectory_timeout",
          &params.trajectory_timeout, "cycle_time", &params.cycle_time,
          "reinitialization_distance", &params.reinitialization_distance,
          "reinitialization_velocity_threshold",
          &params.reinitialization_velocity_threshold)) {
    return EXIT_FAILURE;
  }

  std::mutex clicked_path_mutex;
  bookbot::Path last_recieved_clicked_path;
  ros::Time last_recieved_clicked_path_time;
  bool new_clicked_path_available = false;
  boost::function<void(const planning_msgs::Path&)> clicked_path_callback =
      [&clicked_path_mutex, &last_recieved_clicked_path,
       &last_recieved_clicked_path_time,
       &new_clicked_path_available](const planning_msgs::Path& msg) -> void {
    bookbot::Path new_clicked_path;
    for (const planning_msgs::PathPoint& path_point_msg : msg.points) {
      new_clicked_path.push_back(
          {path_point_msg.distance_along_path, path_point_msg.x,
           path_point_msg.y, path_point_msg.yaw, path_point_msg.curvature});
    }
    std::lock_guard<std::mutex> lock(clicked_path_mutex);
    std::swap(last_recieved_clicked_path, new_clicked_path);
    last_recieved_clicked_path_time = msg.header.stamp;
    new_clicked_path_available = true;
  };
  ros::Subscriber path_subscriber = node_handle.subscribe<planning_msgs::Path>(
      kClickedPathTopic, 10, clicked_path_callback);

  ros::Publisher planned_path_publisher =
      node_handle.advertise<planning_msgs::Path>(kPlannedPathTopic, 1);

  bookbot::TopicListener<nav_msgs::OccupancyGrid> grid_listener(
      kPerceptionTopic, node_handle, 3);
  bookbot::TopicListener<nav_msgs::Odometry> odometry_listener(
      kOdomTopic, node_handle, 500);
  bookbot::TopicListener<planning_msgs::Trajectory> trajectory_listener(
      kTrajectoryTopic, node_handle, 3);

  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::Rate loop_rate(1 / params.cycle_time);
  ros::Time frame_time = ros::Time::now();  // should be perception time
  while (ros::ok()) {
    // Time at which the new path will be published
    ros::Time planning_time = frame_time + ros::Duration(params.cycle_time);

    bool valid_clicked_path = false;
    bookbot::Path clicked_path;
    {
      std::lock_guard<std::mutex> lock(clicked_path_mutex);
      if (!last_recieved_clicked_path.empty() && new_clicked_path_available) {
        std::swap(clicked_path, last_recieved_clicked_path);
        valid_clicked_path = true;
        new_clicked_path_available = false;
      }
    }
    if (!valid_clicked_path ||
        ros::Duration(frame_time - last_recieved_clicked_path_time).toSec() >
            params.clicked_path_timeout) {
      ROS_ERROR("Invalid or stale clicked_path");
      loop_rate.sleep();
      frame_time = ros::Time::now();  // update start time for next cycle
      continue;
    }

    bookbot::PublishIntrospection("/clicked_path", "/odom", clicked_path);

    nav_msgs::Odometry odom;
    if (!odometry_listener.GetClosestBefore(planning_time, &odom) ||
        ros::Duration(frame_time - odom.header.stamp).toSec() >
            params.odometry_timeout) {
      ROS_ERROR("Invalid or stale odometry");
      loop_rate.sleep();
      frame_time = ros::Time::now();  // update start time for next cycle
      continue;
    }

    nav_msgs::OccupancyGrid grid;
    if (params.require_perception &&
        (!grid_listener.GetClosestBefore(planning_time, &grid) ||
         ros::Duration(frame_time - grid.header.stamp).toSec() >
             params.perception_timeout)) {
      ROS_ERROR("Invalid or stale perception");
      loop_rate.sleep();
      frame_time = ros::Time::now();  // update start time for next cycle
      continue;
    }

    bookbot::TimingAggregator timing_aggregator("PrimitivePlannerTiming", true);

    bookbot::SignedDistanceField distance_field;
    if (params.require_perception) {
      bookbot::ScopedTimer timer("GenerateSignedDistanceField");
      distance_field = bookbot::GenerateSignedDistanceField(grid, 1, 30);
    }

    PublishIntrospection("/signed_dist_field", "odom", distance_field, 0.1, 30);

    PublishConfigurationSpaceIntrospection("/configuration_space", "odom",
                                           distance_field, 0.1,
                                           params.robot_radius);

    // Match to previous path
    bookbot::PathPoint initial_state;
    initial_state.distance_along_path = 0;  // Plan new path starting at 0 dist
    initial_state.x = odom.pose.pose.position.x;
    initial_state.y = odom.pose.pose.position.y;
    initial_state.yaw = tf::getYaw(odom.pose.pose.orientation);
    initial_state.curvature = 0.;
    const double current_velocity =
        Eigen::Vector2d(odom.twist.twist.linear.x, odom.twist.twist.linear.y)
            .norm();

    // Check to see if the last recieved trajectory was stitched
    bool last_trajectory_is_stitched = false;
    planning_msgs::Trajectory last_trajectory_msg;
    if (trajectory_listener.GetClosestBefore(planning_time,
                                             &last_trajectory_msg) &&
        ros::Duration(frame_time - last_trajectory_msg.header.stamp).toSec() <
            params.trajectory_timeout &&
        !last_trajectory_msg.points.empty()) {
      last_trajectory_is_stitched = (last_trajectory_msg.trajectory_status ==
                                     planning_msgs::Trajectory::STITCHED);
    }

    double initial_distance_along_path = 0;  // Cache for prefix update
    bookbot::Path prefix;
    if (!last_planned_path.empty() &&
        (last_trajectory_is_stitched ||
         current_velocity > params.reinitialization_velocity_threshold)) {
      // We want to try to stitch to the previous path because we have a
      // previous path to stitch to and either:
      // 1) the robot is in motion, in which case we don't want to abruptly
      // change the path, or
      // 2) the robot is stationary but the planned trajectory is not reiniting
      // meaning that the robot is in the middle of executing a planned maneuver
      auto current_pt = Eigen::Vector2d{initial_state.x, initial_state.y};
      auto bracketing_points = FindClosestSegment(
          current_pt, last_planned_path.begin(), last_planned_path.end());

      auto current_matched_point = MatchBetweenPathPoints(
          current_pt, *bracketing_points.first, *bracketing_points.second);

      if (Eigen::Vector2d(current_matched_point.Position() - current_pt)
              .squaredNorm() > params.reinitialization_distance) {
        ROS_INFO("Primitive Planner is Reiniting");
      } else {
        // Add previous point as prefix for result
        prefix.push_back(*bracketing_points.first);

        // Start at next node point that is sufficiently far ahead
        auto current_node_point_iter = bracketing_points.second;
        while (current_node_point_iter->distance_along_path -
                       current_matched_point.distance_along_path <
                   current_velocity * params.cycle_time &&
               std::next(current_node_point_iter) !=
                   std::end(last_planned_path)) {
          // Add skipped over point as prefix for result
          prefix.push_back(*current_node_point_iter);

          std::advance(current_node_point_iter, 1);
        }
        initial_state.x = current_node_point_iter->x;
        initial_state.y = current_node_point_iter->y;
        initial_state.yaw = current_node_point_iter->yaw;
        initial_state.curvature = current_node_point_iter->curvature;

        // Modify the distance_along_path values in the prefix so that
        // the prefix starts at a distance of 0
        double prefix_start_distance = prefix.front().distance_along_path;
        std::for_each(std::begin(prefix), std::end(prefix),
                      [&prefix_start_distance](bookbot::PathPoint& pt) {
                        pt.distance_along_path -= prefix_start_distance;
                      });

        initial_distance_along_path =
            current_node_point_iter->distance_along_path -
            prefix_start_distance;
      }
    }

    bookbot::SignedDistanceField clicked_path_field;
    {
      bookbot::ScopedTimer timer("GenerateClickedPathField");
      auto distance_function =
          [&clicked_path](Eigen::Vector2d position) -> double {
        auto matched_point = bookbot::MatchToPath(
            position, std::begin(clicked_path), std::end(clicked_path));
        return Eigen::Vector2d(position[0] - matched_point.x,
                               position[1] - matched_point.y)
            .squaredNorm();
      };

      clicked_path_field = bookbot::GenerateSignedDistanceField(
          distance_function, 400, 400, 0.1,
          {initial_state.x - 20, initial_state.y - 20});
    }

    // Generate primitive set given current parameters
    auto primitive_set = bookbot::GeneratePrimitiveSet(
        params.primitive_length, params.primitive_min_turn_radius,
        params.num_primitive_curvature_samples,
        params.max_curvature_change_per_meter);

    // Compute coarse path using a discrete search
    bookbot::Path primitive_path = bookbot::PlanPrimitivePath(
        clicked_path, initial_state, params, primitive_set, distance_field,
        clicked_path_field);

    // Offset planned path by initial_distance_along_path
    std::for_each(std::begin(primitive_path), std::end(primitive_path),
                  [&initial_distance_along_path](bookbot::PathPoint& pt) {
                    pt.distance_along_path += initial_distance_along_path;
                  });

    // Re-add prefix
    primitive_path.insert(std::begin(primitive_path), std::begin(prefix),
                          std::end(prefix));

    // Update last_planned_path
    last_planned_path = primitive_path;

    bookbot::PublishIntrospection("/primitive_path", "/odom", primitive_path);

    planning_msgs::Path planned_path_msg;
    planned_path_msg.header.frame_id = kOdomFrame;
    planned_path_msg.header.stamp = frame_time;
    for (const auto& point : primitive_path) {
      planning_msgs::PathPoint point_msg;
      point_msg.curvature = point.curvature;
      point_msg.distance_along_path = point.distance_along_path;
      point_msg.x = point.x;
      point_msg.y = point.y;
      point_msg.yaw = point.yaw;
      planned_path_msg.points.push_back(point_msg);
    }
    path_publisher.publish(planned_path_msg);

    loop_rate.sleep();
    frame_time = ros::Time::now();  // update start time for next cycle
  }
  return EXIT_SUCCESS;
}

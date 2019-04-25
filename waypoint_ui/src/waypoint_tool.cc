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

#include "waypoint_tool.h"

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSceneManager.h>
#include <geometry_msgs/Point.h>
#include <rviz/geometry.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>

namespace bookbot {

WaypointTool::WaypointTool() : candidate_waypoint_node_(nullptr) {
  shortcut_key_ = 'w';
  waypoint_add_publisher_ =
      node_handle_.advertise<geometry_msgs::Point>(kWaypointAddTopic, 1);
  waypoint_delete_publisher_ =
      node_handle_.advertise<geometry_msgs::Point>(kWaypointDeleteTopic, 1);
  waypoint_reset_publisher_ =
      node_handle_.advertise<geometry_msgs::Point>(kWaypointResetTopic, 1);
}

void WaypointTool::onInitialize() {
  candidate_waypoint_node_ =
      scene_manager_->getRootSceneNode()->createChildSceneNode();
  candidate_waypoint_shape_.reset(new rviz::Shape(
      rviz::Shape::Sphere, scene_manager_, candidate_waypoint_node_));
  candidate_waypoint_shape_->setScale({0.25, 0.25, 0.25});
}

void WaypointTool::activate() {
  if (candidate_waypoint_node_ != nullptr) {
    candidate_waypoint_node_->setVisible(true);
  }
}

void WaypointTool::deactivate() {
  if (candidate_waypoint_node_ != nullptr) {
    candidate_waypoint_node_->setVisible(false);
  }
}

int WaypointTool::processMouseEvent(rviz::ViewportMouseEvent& event) {
  if (candidate_waypoint_node_ == nullptr) {
    return Render;
  }
  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if (!rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x,
                                         event.y, intersection)) {
    candidate_waypoint_node_->setVisible(false);
    return Render;
  }

  if (!event.leftUp()) {
    return Render;
  }

  if (!ros::ok() || !waypoint_add_publisher_ || !waypoint_delete_publisher_ ||
      !waypoint_reset_publisher_) {
    return Render | Finished;
  }

  // Get transformation to odom frame
  tf::StampedTransform odom_T_fixed_frame;
  try {
    tf_listener_.lookupTransform(kOdomFrame,
                                 context_->getFixedFrame().toStdString(),
                                 ros::Time(0), odom_T_fixed_frame);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return Render;
  }

  tf::Vector3 point_in_odom_frame =
      odom_T_fixed_frame *
      tf::Vector3(intersection.x, intersection.y, intersection.z);

  geometry_msgs::Point point_msg;
  point_msg.x = point_in_odom_frame.x();
  point_msg.y = point_in_odom_frame.y();
  point_msg.z = point_in_odom_frame.z();

  if (event.shift()) {
    waypoint_add_publisher_.publish(point_msg);
  } else if (event.control()) {
    waypoint_reset_publisher_.publish(point_msg);
  } else if (event.alt()) {
    waypoint_delete_publisher_.publish(point_msg);
  }

  return Render | Finished;
}

}  // namespace bookbot

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(bookbot::WaypointTool, rviz::Tool)

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

#ifndef BOOKBOT_WAYPOINT_UI_WAYPOINT_ADD_TOOL_H_
#define BOOKBOT_WAYPOINT_UI_WAYPOINT_ADD_TOOL_H_

#include <OGRE/OgreSceneNode.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/tool.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace bookbot {
constexpr char kWaypointAddTopic[] = "/waypoint/add";
constexpr char kWaypointDeleteTopic[] = "/waypoint/delete";
constexpr char kWaypointResetTopic[] = "/waypoint/reset";
constexpr char kOdomFrame[] = "odom";

class WaypointTool : public rviz::Tool {
  Q_OBJECT
 public:
  WaypointTool();

  void onInitialize() override;
  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz::ViewportMouseEvent& event) override;

 private:
  // rviz cleans up SceneNode pointers itself so a RAII pointer is not an
  // option
  Ogre::SceneNode* candidate_waypoint_node_;
  std::unique_ptr<rviz::Shape> candidate_waypoint_shape_;
  ros::NodeHandle node_handle_;
  ros::Publisher waypoint_add_publisher_;
  ros::Publisher waypoint_delete_publisher_;
  ros::Publisher waypoint_reset_publisher_;
  tf::TransformListener tf_listener_;
};

}  // namespace bookbot

#endif  // SIDEWALKER_WAYPOINT_UI_WAYPOINT_ADD_TOOL_H_

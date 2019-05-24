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

#include <primitive_planner/primitive_search_node.h>

namespace bookbot {

PathPoint ApplyPathPrimitive(const PathPrimitive& primitive,
                             const PathPoint& start_point) {
  Eigen::Vector2d relative_position =
      Eigen::Rotation2Dd(start_point.yaw) *
      Eigen::Vector2d(primitive.x_rel, primitive.y_rel);
  return {start_point.distance_along_path + primitive.length,
          start_point.x + relative_position[0],
          start_point.y + relative_position[1],
          start_point.yaw + primitive.delta_yaw, primitive.end_curvature};
}

PathPrimitiveSearchNode::PathPrimitiveSearchNode(
    double x_in, double y_in, double yaw_in, double curvature_in,
    double distance_traveled_in, int tree_depth_in,
    PathPrimitive previous_primitive_in, double cost_from_start_in,
    double cost_to_goal_in, int parent_index_in)
    : x(x_in),
      y(y_in),
      yaw(yaw_in),
      curvature(curvature_in),
      distance_traveled(distance_traveled_in),
      tree_depth(tree_depth_in),
      previous_primitive(previous_primitive_in),
      cost_from_start(cost_from_start_in),
      cost_to_goal(cost_to_goal_in),
      parent_index(parent_index_in) {}

void PathPrimitiveSearchNode::SetIntrospection(
    double distance_to_obstacles, double obst_cost, double path_cost,
    double curve_cost, double travel_cost, double incremental_node_cost) {
  introspection_data.SetIntrospection(distance_to_obstacles, obst_cost,
                                      path_cost, curve_cost, travel_cost,
                                      incremental_node_cost);
}

std::string PathPrimitiveSearchNode::GetIntrospectionDescription() {
  return introspection_data.GetIntrospectionDescription();
}

}  // namespace bookbot

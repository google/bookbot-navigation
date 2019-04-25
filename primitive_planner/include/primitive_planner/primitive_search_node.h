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

#ifndef BOOKBOT_PRIMITIVE_PLANNER_PRIMITIVE_SEARCH_NODE_H_
#define BOOKBOT_PRIMITIVE_PLANNER_PRIMITIVE_SEARCH_NODE_H_

#include <primitive_planner/primitive.h>

#include <string>

constexpr bool kUseNodeIntrospection = false;

namespace bookbot {

/**
 * @brief Applies a precomputed path primitive to a given initial state
 *
 * @param [in] primitive        The path primitive being applied
 * @param [in] start_point      The starting point from which the path primitive
 *                              is applied
 * @return PathPoint            The final state after applying the primitive
 */
PathPoint ApplyPathPrimitive(const PathPrimitive& primitive,
                             const PathPoint& start_point);

// Conditional introspection data struct which specializes to empty if the
// template parameter is false
template <bool Condition>
struct IntrospectionInfo {
  void SetIntrospection(double distance_to_obstacles_in, double obst_cost_in,
                        double path_cost_in, double curve_cost_in,
                        double travel_cost_in,
                        double incremental_node_cost_in) {
    distance_to_obstacles = distance_to_obstacles_in;
    obst_cost = obst_cost_in;
    path_cost = path_cost_in;
    curve_cost = curve_cost_in;
    travel_cost = travel_cost_in;
    incremental_node_cost = incremental_node_cost_in;
  }

  std::string GetIntrospectionDescription() {
    std::stringstream ss;
    ss << "dist_to_obst:" << distance_to_obstacles << ",obst_cost:" << obst_cost
       << ",path_cost:" << path_cost << ",trav_cost:" << travel_cost
       << ",curve_cost:" << curve_cost
       << ",incr_cost:" << incremental_node_cost;
    return ss.str();
  }

  double distance_to_obstacles;
  double obst_cost;
  double path_cost;
  double curve_cost;
  double travel_cost;
  double incremental_node_cost;
};
template <>
struct IntrospectionInfo<false> {
  void SetIntrospection(double distance_to_obstacles_in, double obst_cost_in,
                        double path_cost_in, double curve_cost_in,
                        double travel_cost_in,
                        double incremental_node_cost_in) {}

  std::string GetIntrospectionDescription() { return ""; }
};

class PathPrimitiveSearchNode {
 public:
  PathPrimitiveSearchNode(double x_in, double y_in, double yaw_in,
                          double distance_traveled_in, int tree_depth_in,
                          PathPrimitive previous_primitive_in,
                          double cost_from_start_in, double cost_to_goal_in,
                          int parent_index_in);

  inline Eigen::Vector2d Position() const { return {x, y}; }

  inline PathPoint State() const { return {distance_traveled, x, y, yaw, 0.}; }

  double x;
  double y;
  double yaw;
  double distance_traveled;
  int tree_depth;
  PathPrimitive previous_primitive;
  double cost_from_start;
  double cost_to_goal;
  int parent_index;

  //----------------------------------------------------------------------------
  // Introspection
  //----------------------------------------------------------------------------

  void SetIntrospection(double distance_to_obstacles, double obst_cost,
                        double path_cost, double curve_cost, double travel_cost,
                        double incremental_node_cost);

  std::string GetIntrospectionDescription();

 private:
  IntrospectionInfo<kUseNodeIntrospection> introspection_data;
};

}  // namespace bookbot

#endif  // BOOKBOT_PRIMITIVE_PLANNER_PRIMITIVE_SEARCH_NODE_H_

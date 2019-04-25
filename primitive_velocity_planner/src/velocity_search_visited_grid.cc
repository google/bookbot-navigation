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

#include <velocity_planner/velocity_search_visited_grid.h>

#include <cmath>

namespace bookbot {

bool VelocitySearchVisitedGrid::VelocitySearchVisitedGridNode::operator==(
    const VelocitySearchVisitedGridNode& other) const {
  return time_index == other.time_index &&
         velocity_index == other.velocity_index &&
         distance_along_path_index == other.distance_along_path_index;
}

bool VelocitySearchVisitedGrid::VelocitySearchVisitedGridNode::operator<(
    const VelocitySearchVisitedGridNode& other) const {
  return (time_index == other.time_index)
             ? (velocity_index == other.velocity_index
                    ? distance_along_path_index <
                          other.distance_along_path_index
                    : velocity_index < other.velocity_index)
             : time_index < other.time_index;
}

int cantor_pairing_function(int a, int b) {
  // https://en.wikipedia.org/wiki/Pairing_function#Cantor_pairing_function
  return 0.5 * (a + b) * (a + b + 1) + b;
}

// clang-format off
std::size_t
VelocitySearchVisitedGrid::VelocitySearchVisitedGridNodeHash::operator()(
    const VelocitySearchVisitedGridNode& node) const {
  return std::hash<int>()(cantor_pairing_function(
      cantor_pairing_function(node.time_index, node.velocity_index),
      node.distance_along_path_index));
}
// clang-format on

VelocitySearchVisitedGrid::VelocitySearchVisitedGrid(
    double distance_along_path_resolution)
    : distance_along_path_resolution_(distance_along_path_resolution) {}

bool VelocitySearchVisitedGrid::Contains(int time_index, int velocity_index,
                                         double distance_along_path) const {
  auto visited_list_iter = visited_list_.find(
      GenerateGridNode(time_index, velocity_index, distance_along_path));
  return visited_list_iter != std::end(visited_list_);
}

bool VelocitySearchVisitedGrid::Insert(int time_index, int velocity_index,
                                       double distance_along_path) {
  auto insert_result = visited_list_.insert(
      GenerateGridNode(time_index, velocity_index, distance_along_path));
  return insert_result.second;
}

VelocitySearchVisitedGrid::VelocitySearchVisitedGridNode
VelocitySearchVisitedGrid::GenerateGridNode(
    int time_index_in, int velocity_index_in,
    double distance_along_path_in) const {
  int distance_along_path_index = static_cast<int>(
      distance_along_path_in / distance_along_path_resolution_);
  return {time_index_in, velocity_index_in, distance_along_path_index};
};

}  // namespace bookbot

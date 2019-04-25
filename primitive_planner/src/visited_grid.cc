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

#include <primitive_planner/visited_grid.h>

#include <cmath>

namespace bookbot {

bool VisitedGrid::VisitedGridNode::operator==(
    const VisitedGridNode& other) const {
  return x_index == other.x_index && y_index == other.y_index &&
         angle_bin == other.angle_bin;
}

bool VisitedGrid::VisitedGridNode::operator<(
    const VisitedGridNode& other) const {
  return (x_index == other.x_index)
             ? (y_index == other.y_index ? angle_bin < other.angle_bin
                                         : y_index < other.y_index)
             : x_index < other.x_index;
}

int cantor_pairing_function(int a, int b) {
  // https://en.wikipedia.org/wiki/Pairing_function#Cantor_pairing_function
  return 0.5 * (a + b) * (a + b + 1) + b;
}

std::size_t VisitedGrid::VisitedGridNodeHash::operator()(
    const VisitedGridNode& node) const {
  return std::hash<int>()(cantor_pairing_function(
      cantor_pairing_function(node.x_index, node.y_index), node.angle_bin));
}

VisitedGrid::VisitedGrid(double spatial_resolution,
                         int num_visited_grid_angle_bins)
    : spatial_resolution_(spatial_resolution),
      num_visited_grid_angle_bins_(num_visited_grid_angle_bins) {}

bool VisitedGrid::Contains(double x, double y, double yaw) {
  auto visited_list_iter = visited_list_.find(GenerateGridNode(x, y, yaw));
  return visited_list_iter != std::end(visited_list_);
}

bool VisitedGrid::Insert(double x, double y, double yaw) {
  auto insert_result = visited_list_.insert(GenerateGridNode(x, y, yaw));
  return insert_result.second;
}

VisitedGrid::VisitedGridNode VisitedGrid::GenerateGridNode(double x, double y,
                                                           double yaw) {
  VisitedGridNode visited_node;
  // bin x and y values
  visited_node.x_index = x / spatial_resolution_;
  visited_node.y_index = y / spatial_resolution_;
  // bin yaw angle
  const double normalized_yaw = std::remainder(yaw, 2 * M_PI);
  const double wrapped_yaw =
      normalized_yaw < 0 ? normalized_yaw + 2.0 * M_PI : normalized_yaw;
  const int angle_bin =
      static_cast<int>((wrapped_yaw + M_PI / num_visited_grid_angle_bins_) /
                       (2.0 * M_PI / num_visited_grid_angle_bins_));
  visited_node.angle_bin =
      (angle_bin == num_visited_grid_angle_bins_) ? 0 : angle_bin;
  return visited_node;
}

}  // namespace bookbot

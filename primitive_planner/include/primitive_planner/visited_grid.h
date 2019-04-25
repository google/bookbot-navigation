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

#ifndef BOOKBOT_PRIMITIVE_PLANNER_VISITED_GRID_H_
#define BOOKBOT_PRIMITIVE_PLANNER_VISITED_GRID_H_

#include <unordered_set>

namespace bookbot {

class VisitedGrid {
 public:
  VisitedGrid(double spatial_resolution, int num_visited_grid_angle_bins);

  bool Contains(double x, double y, double yaw);

  bool Insert(double x, double y, double yaw);

 private:
  struct VisitedGridNode {
    bool operator==(const VisitedGridNode& other) const;
    bool operator<(const VisitedGridNode& other) const;

    int x_index;
    int y_index;
    int angle_bin;
  };

  struct VisitedGridNodeHash {
    std::size_t operator()(const VisitedGridNode& node) const;
  };

  VisitedGridNode GenerateGridNode(double x, double y, double yaw);

  double spatial_resolution_;
  int num_visited_grid_angle_bins_;
  std::unordered_set<VisitedGridNode, VisitedGridNodeHash> visited_list_;
};

}  // namespace bookbot

#endif  // BOOKBOT_PRIMITIVE_PLANNER_VISITED_GRID_H_

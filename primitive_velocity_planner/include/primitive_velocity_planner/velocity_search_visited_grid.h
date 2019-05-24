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

#ifndef BOOKBOT_PRIMITIVE_VELOCITY_PLANNER_VISITED_GRID_H_
#define BOOKBOT_PRIMITIVE_VELOCITY_PLANNER_VISITED_GRID_H_

#include <unordered_set>

namespace bookbot {

class VelocitySearchVisitedGrid {
 public:
  VelocitySearchVisitedGrid(double distance_along_path_resolution);

  bool Contains(int time_index, int velocity_index,
                double distance_along_path) const;

  bool Insert(int time_index, int velocity_index, double distance_along_path);

 private:
  struct VelocitySearchVisitedGridNode {
    bool operator==(const VelocitySearchVisitedGridNode& other) const;
    bool operator<(const VelocitySearchVisitedGridNode& other) const;

    int time_index;
    int velocity_index;
    int distance_along_path_index;
  };

  struct VelocitySearchVisitedGridNodeHash {
    std::size_t operator()(const VelocitySearchVisitedGridNode& node) const;
  };

  VelocitySearchVisitedGridNode GenerateGridNode(
      int time_index_in, int velocity_index_in,
      double distance_along_path_in) const;

  double distance_along_path_resolution_;
  std::unordered_set<VelocitySearchVisitedGridNode,
                     VelocitySearchVisitedGridNodeHash>
      visited_list_;
};

}  // namespace bookbot

#endif  // BOOKBOT_PRIMITIVE_VELOCITY_PLANNER_VISITED_GRID_H_

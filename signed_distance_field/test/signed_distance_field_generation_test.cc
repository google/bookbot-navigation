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

#include <gtest/gtest.h>
#include <nav_msgs/OccupancyGrid.h>
#include <signed_distance_field/signed_distance_field.h>

TEST(SignedDistanceFieldGenerationTest, FromOccupancyGridTest) {
  nav_msgs::OccupancyGrid occupancy_grid_message;
  occupancy_grid_message.info.origin.position.x = -1;
  occupancy_grid_message.info.origin.position.y = -1;
  occupancy_grid_message.info.origin.orientation.w = 1;
  occupancy_grid_message.info.resolution = 0.5;
  occupancy_grid_message.info.width = 10;
  occupancy_grid_message.info.height = 10;
  // clang-format off
  occupancy_grid_message.data = {100, 100, 100, 0,   0,   0,   0,   0,   0,  0,
                                 100, 100, 100, 0,   0,   0,   0,   0,   0,  0,
                                 100, 100, 100, 0,   0,   0,   0,   0,   0,  0,
                                 0,   0,   0,   0,   0,   0,   0,   0,   0,  0,
                                 0,   0,   0,   0,   0,   0,   0,   0,   0,  0,
                                 0,   0,   0,   100, 100, 100, 0,   0,   0,  0,
                                 0,   0,   100, 100, 100, 100, 100, 0,   0,  0,
                                 0,   0,   100, 100, 100, 100, 100, 0,   0,  0,
                                 0,   0,   0,   100, 100, 100, 100, 0,   0,  0,
                                 0,   0,   0,     0, 100, 100, 100, 0,   0,  0};
  // clang-format on

  auto distance_field =
      bookbot::GenerateSignedDistanceField(occupancy_grid_message, 2, 30);
  std::cout << distance_field << std::endl;
  EXPECT_EQ(distance_field.Evaluate({-1, -1}), -1.5);
  EXPECT_EQ(distance_field.Evaluate({1, 0}), 0.5);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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
#include <trajectory_math/path_algorithms.h>

TEST(TestMatchToPath, BasicMatchTest) {
  bookbot::Path path;
  path.push_back({0, 0, 0, 0, 0});
  path.push_back({5, 5, 0, 0, 0});
  path.push_back({10, 5, 5, M_PI / 2, 0.5});
  bookbot::PathPoint matched_point =
      bookbot::MatchToPath({6, 3}, std::begin(path), std::end(path));
  EXPECT_EQ(matched_point.distance_along_path, 8);
  EXPECT_EQ(matched_point.x, 5);
  EXPECT_EQ(matched_point.y, 3);
  EXPECT_EQ(matched_point.yaw, 3 * M_PI / 10);
  EXPECT_EQ(matched_point.curvature, 0.3);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

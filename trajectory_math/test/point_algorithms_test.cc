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
#include <trajectory_math/point_algorithms.h>

TEST(TestPointAlgorithms, AreaTest) {
  {
    const Eigen::Vector2d point1 = {0, 0};
    const Eigen::Vector2d point2 = {5, 0};
    const Eigen::Vector2d point3 = {0, 5};
    EXPECT_EQ(bookbot::TriangleArea(point2, point1, point3), 12.5);
  }
  {
    const Eigen::Vector2d point1 = {0, 0};
    const Eigen::Vector2d point2 = {5, 0};
    const Eigen::Vector2d point3 = {5, 5};
    EXPECT_EQ(bookbot::TriangleArea(point2, point1, point3), 12.5);
  }
  {
    const Eigen::Vector2d point1 = {0, 0};
    const Eigen::Vector2d point2 = {5, 0};
    const Eigen::Vector2d point3 = {2.5, 2.5};
    EXPECT_EQ(bookbot::TriangleArea(point2, point1, point3), 6.25);
  }
  {
    const Eigen::Vector2d point1 = {0, 0};
    const Eigen::Vector2d point2 = {5, 7};
    const Eigen::Vector2d point3 = {7, 5};
    EXPECT_EQ(bookbot::TriangleArea(point2, point1, point3), 12);
  }
  {
    const Eigen::Vector2d point1 = {0, 0};
    const Eigen::Vector2d point2 = {-5, 7};
    const Eigen::Vector2d point3 = {7, 5};
    EXPECT_EQ(bookbot::TriangleArea(point2, point1, point3), 37);
  }
}

TEST(TestPointAlgorithms, CurvatureTest) {
  {
    const Eigen::Vector2d point1 = {0, 0};
    const Eigen::Vector2d point2 = {1, 1};
    const Eigen::Vector2d point3 = {0, 2};
    EXPECT_EQ(bookbot::ApproximateCurvatureMagnitude(point2, point1, point3),
              1);
  }
  {
    const Eigen::Vector2d point1 = {0, 0};
    const Eigen::Vector2d point2 = {5, 0};
    const Eigen::Vector2d point3 = {-5, 0};
    EXPECT_EQ(bookbot::ApproximateCurvatureMagnitude(point2, point1, point3),
              0);
  }
  {
    const Eigen::Vector2d point1 = {0, 0};
    const Eigen::Vector2d point2 = {5, 5};
    const Eigen::Vector2d point3 = {0, 10};
    EXPECT_EQ(bookbot::ApproximateCurvatureMagnitude(point2, point1, point3),
              1. / 5.);
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

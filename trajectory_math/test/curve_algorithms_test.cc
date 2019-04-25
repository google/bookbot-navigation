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
#include <trajectory_math/curve_algorithms.h>

TEST(TestLinearAngleInterpolation, BasicTest) {
  EXPECT_FLOAT_EQ(bookbot::LinearAngleInterpolation(0, M_PI / 2, 0.5),
                  M_PI / 4);
  EXPECT_FLOAT_EQ(bookbot::LinearAngleInterpolation(0, -M_PI / 2, 0.5),
                  -M_PI / 4);
  EXPECT_FLOAT_EQ(
      bookbot::LinearAngleInterpolation(M_PI / 4, 7 * M_PI / 4, 0.75),
      -M_PI / 8);
  EXPECT_FLOAT_EQ(
      bookbot::LinearAngleInterpolation(3 * M_PI / 4, 3 * M_PI / 4, 0.1),
      3 * M_PI / 4);
  EXPECT_FLOAT_EQ(bookbot::LinearAngleInterpolation(0, -7.5 * M_PI, 0.75),
                  3 * M_PI / 8);
}

TEST(TestSquaredDistanceToLineSegment, BasicTest) {
  {
    Eigen::Vector2d first_pt = {0, 0};
    Eigen::Vector2d second_pt = {10, 0};
    Eigen::Vector2d query_pt = {5, 5};

    double interp_fraction;
    double squared_dist_to_line_segment = bookbot::SquaredDistanceToLineSegment(
        query_pt, first_pt, second_pt, &interp_fraction);
    EXPECT_FLOAT_EQ(squared_dist_to_line_segment, 25);
    EXPECT_FLOAT_EQ(interp_fraction, 0.5);
  }

  {
    Eigen::Vector2d first_pt = {0, 0};
    Eigen::Vector2d second_pt = {-10, -10};
    Eigen::Vector2d query_pt = {-10, 0};

    double interp_fraction;
    double squared_dist_to_line_segment = bookbot::SquaredDistanceToLineSegment(
        query_pt, first_pt, second_pt, &interp_fraction);
    EXPECT_FLOAT_EQ(squared_dist_to_line_segment, 50);
    EXPECT_FLOAT_EQ(interp_fraction, 0.5);
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

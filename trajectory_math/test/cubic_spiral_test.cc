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
#include <trajectory_math/cubic_spiral.h>

TEST(TestCubicSpiral, CircleEvaluationTest) {
  bookbot::PathPoint initial_state;
  initial_state.x = 0;
  initial_state.y = 0;
  initial_state.yaw = 0;
  initial_state.curvature = 0.25;  // circle of radius r=4
  initial_state.distance_along_path = 0;

  auto evaluated_point =
      bookbot::InterpolateCubicSpiralSegmentByDistanceAlongPath(
          initial_state, 4 * M_PI, M_PI, 0.25, 2 * M_PI);
  EXPECT_FLOAT_EQ(evaluated_point.x, 4);
  EXPECT_FLOAT_EQ(evaluated_point.y, 4);
  EXPECT_FLOAT_EQ(evaluated_point.yaw, M_PI_2);
  EXPECT_FLOAT_EQ(evaluated_point.distance_along_path, 2 * M_PI);
  EXPECT_FLOAT_EQ(evaluated_point.curvature, 0.25);
}

TEST(TestCubicSpiral, OffsetClothoidEvaluationTest) {
  bookbot::PathPoint initial_state;
  initial_state.x = 1;
  initial_state.y = 3;
  initial_state.yaw = 1;
  initial_state.curvature = 1;
  initial_state.distance_along_path = 1;

  // Assume curve of form (where s=0 at start of segment):
  // \kappa(s) = s + 1
  // \theta(s) = 0.5*s^2 + s + 1

  const double end_curvature = 3;
  const double end_yaw = 5;
  const double end_distance_along_path = 3;  // For a segment length of 2

  auto evaluated_point =
      bookbot::InterpolateCubicSpiralSegmentByDistanceAlongPath(
          initial_state,
          end_distance_along_path - initial_state.distance_along_path, end_yaw,
          end_curvature, 1);
  EXPECT_FLOAT_EQ(evaluated_point.x, 0.9160497);
  EXPECT_FLOAT_EQ(evaluated_point.y, 3.9043727);
  EXPECT_FLOAT_EQ(evaluated_point.yaw, 2.5);
  EXPECT_FLOAT_EQ(evaluated_point.distance_along_path, 2);
  EXPECT_FLOAT_EQ(evaluated_point.curvature, 2);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

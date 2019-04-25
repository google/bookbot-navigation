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
#include <trajectory_math/cubic_hermite_interpolation.h>

using Point = std::array<double, 3>;  // {t, s, s_dot}

TEST(TestCubicHermiteInterpolation, ConstAccelTest) {
  Point start = {0, 0, 0};
  Point end = {1, 1, 2};

  // interpolate at start
  double t_interpolate = 0;
  EXPECT_EQ(bookbot::CubicHermiteInterpolate(t_interpolate, start[0], start[1],
                                             start[2], end[0], end[1], end[2]),
            0);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      0);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateSecondDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      2);

  // interpolate at end
  t_interpolate = 1;
  EXPECT_EQ(bookbot::CubicHermiteInterpolate(t_interpolate, start[0], start[1],
                                             start[2], end[0], end[1], end[2]),
            1);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      2);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateSecondDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      2);

  // interpolate in middle
  t_interpolate = 0.5;
  EXPECT_EQ(bookbot::CubicHermiteInterpolate(t_interpolate, start[0], start[1],
                                             start[2], end[0], end[1], end[2]),
            0.25);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      1);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateSecondDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      2);
}

TEST(TestCubicHermiteInterpolation, CubicTest) {
  // testing polynomial s(t) = 0.5*t^3 + 2*t^2 + t + 5
  Point start = {0, 5, 1};
  Point end = {1, 8.5, 6.5};

  // interpolate at start
  double t_interpolate = 0;
  EXPECT_EQ(bookbot::CubicHermiteInterpolate(t_interpolate, start[0], start[1],
                                             start[2], end[0], end[1], end[2]),
            5);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      1);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateSecondDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      4);

  // interpolate at end
  t_interpolate = 1;
  EXPECT_EQ(bookbot::CubicHermiteInterpolate(t_interpolate, start[0], start[1],
                                             start[2], end[0], end[1], end[2]),
            8.5);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      6.5);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateSecondDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      7);

  // interpolate in middle
  t_interpolate = 0.5;
  EXPECT_EQ(bookbot::CubicHermiteInterpolate(t_interpolate, start[0], start[1],
                                             start[2], end[0], end[1], end[2]),
            6.0625);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      3.375);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateSecondDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      5.5);
}

TEST(TestCubicHermiteInterpolation, ZeroAccelNonUnitIntervalTest) {
  Point start = {0, 0, 1};
  Point end = {3.5, 3.5, 1};

  // interpolate at start
  double t_interpolate = 0;
  EXPECT_EQ(bookbot::CubicHermiteInterpolate(t_interpolate, start[0], start[1],
                                             start[2], end[0], end[1], end[2]),
            0);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      1);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateSecondDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      0);

  // interpolate at end
  t_interpolate = 3.5;
  EXPECT_EQ(bookbot::CubicHermiteInterpolate(t_interpolate, start[0], start[1],
                                             start[2], end[0], end[1], end[2]),
            3.5);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      1);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateSecondDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      0);

  // interpolate in middle
  t_interpolate = 1.75;
  EXPECT_EQ(bookbot::CubicHermiteInterpolate(t_interpolate, start[0], start[1],
                                             start[2], end[0], end[1], end[2]),
            1.75);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      1);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateSecondDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      0);
}

TEST(TestCubicHermiteInterpolation, CubicNonUnitIntervalTest) {
  // testing polynomial s(t) = 0.1*t^3 + 0.5*t^2 + t + 1
  Point start = {0, 1, 1};
  Point end = {2.5, 8.1875, 5.375};

  // interpolate at start
  double t_interpolate = 0;
  EXPECT_EQ(bookbot::CubicHermiteInterpolate(t_interpolate, start[0], start[1],
                                             start[2], end[0], end[1], end[2]),
            1);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      1);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateSecondDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      1);

  // interpolate at end
  t_interpolate = 2.5;
  EXPECT_EQ(bookbot::CubicHermiteInterpolate(t_interpolate, start[0], start[1],
                                             start[2], end[0], end[1], end[2]),
            8.1875);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      5.375);
  EXPECT_EQ(
      bookbot::CubicHermiteInterpolateSecondDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      2.5);

  // interpolate in middle
  t_interpolate = 1.1;
  EXPECT_FLOAT_EQ(
      bookbot::CubicHermiteInterpolate(t_interpolate, start[0], start[1],
                                       start[2], end[0], end[1], end[2]),
      2.8381);
  EXPECT_FLOAT_EQ(
      bookbot::CubicHermiteInterpolateDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      2.463);
  EXPECT_FLOAT_EQ(
      bookbot::CubicHermiteInterpolateSecondDerivative(
          t_interpolate, start[0], start[1], start[2], end[0], end[1], end[2]),
      1.66);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

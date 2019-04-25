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
#include <signed_distance_field/signed_distance_field.h>

class SignedDistanceFieldTest : public ::testing::Test {
 protected:
  void SetUp() override {
    Eigen::Matrix<double, 4, 4> field;
    // clang-format off
    field << 0.1, 0.2, 0.3, 0.4,
             10.2, 10.4, 10.6, 10.8,
             20.4, 20.8, 21.2, 21.6,
             30.8, 31.6, 32.4, 33.2;
    // clang-format on

    auto origin = Eigen::Vector2d(-3, -3);
    double resolution = 2.0;
    test_field_ = bookbot::SignedDistanceField(resolution, origin, field);
  }

  bookbot::SignedDistanceField test_field_;
};

TEST_F(SignedDistanceFieldTest, EvaluationTest) {
  EXPECT_FLOAT_EQ(test_field_.Evaluate(Eigen::Vector2d(-4.0, -4.0)),
                  kMaxDistance);
  EXPECT_FLOAT_EQ(test_field_.Evaluate(Eigen::Vector2d(-3.0, -3.0)), 0.1);
  EXPECT_FLOAT_EQ(test_field_.Evaluate(Eigen::Vector2d(2.0, -2.0)), 25.9);
}

TEST_F(SignedDistanceFieldTest, JacobianTest) {
  Eigen::Vector2d Jacobian =
      test_field_.EvaluateJacobian(Eigen::Vector2d(2.0, -2.0));
  double h = 0.1;
  double finite_diff_dx =
      (test_field_.Evaluate(Eigen::Vector2d(2.0 + h, -2.0)) -
       test_field_.Evaluate(Eigen::Vector2d(2.0, -2.0))) /
      h;
  double finite_diff_dy =
      (test_field_.Evaluate(Eigen::Vector2d(2.0, -2.0 + h)) -
       test_field_.Evaluate(Eigen::Vector2d(2.0, -2.0))) /
      h;
  EXPECT_FLOAT_EQ(Jacobian[0], finite_diff_dx);
  EXPECT_FLOAT_EQ(Jacobian[1], finite_diff_dy);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

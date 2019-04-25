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
#include <trajectory_math/gauss_legendre_integration.h>

TEST(TestGaussLegendreIntegration, CubicTest) {
  auto polynomial = [](double x) -> double {
    return 12. + x * (-1. + x * (0.5 + x * 3.));  // y = 3x^3 + 0.5x^2 - x + 12
  };

  auto polynomial_integral = [](double x) {
    // int(y) = 3/4x^4 + 1/6x^3 - 1/2x^2 + 12x + C
    return x * (12. + x * (-1. / 2. + x * (1. / 6. + x * 3. / 4.)));
  };

  EXPECT_FLOAT_EQ(bookbot::GaussLegendreIntegrate(polynomial),
                  polynomial_integral(1) - polynomial_integral(-1));

  EXPECT_FLOAT_EQ(bookbot::GaussLegendreIntegrate(polynomial, -13, 15),
                  polynomial_integral(15) - polynomial_integral(-13));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

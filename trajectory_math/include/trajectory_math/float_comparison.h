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

#ifndef BOOKBOT_TRAJECTORY_MATH_FLOAT_COMPARISON_H_
#define BOOKBOT_TRAJECTORY_MATH_FLOAT_COMPARISON_H_

#include <cmath>
#include <cstdlib>
#include <limits>
#include <type_traits>

namespace bookbot {

template <typename RealType>
struct Epsilon {
  static_assert(std::is_floating_point<RealType>::value,
                "Epsilon type is not a floating point type");
  static constexpr RealType value = RealType(1e-6);
};

template <typename RealType>
bool ApproxZero(RealType a, RealType epsilon = Epsilon<RealType>::value) {
  return std::abs(a) < epsilon;
}

template <typename RealType>
bool ApproxEqual(RealType a, RealType b,
                 RealType epsilon = Epsilon<RealType>::value) {
  return ApproxZero(std::abs(a - b), epsilon);
}

}  // namespace bookbot

#endif  // BOOKBOT_TRAJECTORY_MATH_FLOAT_COMPARISON_H_

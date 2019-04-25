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

#ifndef BOOKBOT_ROS_UTILITIES_SCOPED_TIMER_H_
#define BOOKBOT_ROS_UTILITIES_SCOPED_TIMER_H_

#include <absl/strings/string_view.h>
#include <absl/time/time.h>

namespace bookbot {

/**
 * @brief RAII object that captures all scoped timer instances within its thread
 * and scope and publishes a diagnostic message with timing stats upon
 * destruction
 *
 * @note If this is called within the scope of another TimingAggregator object
 * it is silently ignored, allowing the outer-scoped TimingAggregator to capture
 * everything
 */
struct TimingAggregator {
  TimingAggregator(absl::string_view timing_group_name,
                   bool print_result = false);
  ~TimingAggregator();

 private:
  bool valid_;
};

/**
 * @brief RAII object that measures the time that is taken by its scope.
 *
 * @note The ScopedTimer object must exist within the scope of a
 * TimingAggregator object to have any effect.
 */
struct ScopedTimer {
  ScopedTimer(absl::string_view timer_name);
  ~ScopedTimer();

 private:
  absl::string_view timer_name_;
  absl::Time start_time_;
};

}  // namespace bookbot

#endif  // BOOKBOT_ROS_UTILITIES_SCOPED_TIMER_H_

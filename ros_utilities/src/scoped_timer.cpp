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

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <ros/time.h>
#include <ros_utilities/introspection.h>
#include <ros_utilities/scoped_timer.h>

#include <map>

constexpr char kDiagnosticsTopic[] = "/diagnostics";

namespace bookbot {

namespace {
struct TimingStatistics {
  int number_of_calls;
  double average_time_in_milliseconds;
  double min_time_in_milliseconds;
  double max_time_in_milliseconds;
};

std::ostream& operator<<(std::ostream& stream,
                         const TimingStatistics& timing_statistics) {
  stream << "[ tot:"
         << timing_statistics.number_of_calls *
                timing_statistics.average_time_in_milliseconds
         << " num:" << timing_statistics.number_of_calls
         << " avg:" << timing_statistics.average_time_in_milliseconds
         << " max:" << timing_statistics.max_time_in_milliseconds
         << " min:" << timing_statistics.min_time_in_milliseconds << "]";
  return stream;
}

struct TimingBackend {
  TimingBackend() : valid(false), print_result(false) {}
  TimingBackend(absl::string_view group_name, bool print_flag)
      : timing_group_name(group_name), valid(true), print_result(print_flag) {}

  absl::string_view timing_group_name;
  std::map<absl::string_view, TimingStatistics> timers;
  bool valid;
  bool print_result;
};

std::ostream& operator<<(std::ostream& stream,
                         const TimingBackend& timing_backend) {
  stream << "Timing Results for:" << timing_backend.timing_group_name
         << " in milliseconds" << std::endl;
  for (const auto& timer_pair : timing_backend.timers) {
    stream << timer_pair.first << ":" << timer_pair.second << std::endl;
  }
  return stream;
}

void PublishTimingIntrospection(const TimingBackend& timing_backend) {
  absl::optional<IntrospectionBackend>& backend = GetIntrospectionBackend();
  if (!backend.has_value()) {
    return;
  }

  diagnostic_msgs::DiagnosticArray timing_msg;
  timing_msg.header.stamp = ros::Time::now();

  diagnostic_msgs::DiagnosticStatus status;
  status.level = diagnostic_msgs::DiagnosticStatus::OK;
  std::stringstream ss;
  ss << timing_backend;
  status.message = ss.str();

  timing_msg.status.push_back(status);

  backend.value().Publish(kDiagnosticsTopic, std::move(timing_msg));
}

TimingBackend& AccessTimingBackend(TimingBackend* reset_value) {
  static thread_local TimingBackend timing_backend;
  if (reset_value) {
    timing_backend = std::move(*reset_value);
  }
  return timing_backend;
}

TimingBackend& GetTimingBackend() { return AccessTimingBackend(nullptr); }

}  // anonymous namespace

ScopedTimer::ScopedTimer(absl::string_view timer_name)
    : timer_name_(timer_name) {
  start_time_ = absl::Now();
}

ScopedTimer::~ScopedTimer() {
  auto end_time = absl::Now();
  double measured_duration_in_milliseconds =
      absl::ToDoubleMilliseconds(end_time - start_time_);

  auto& timing_backend = GetTimingBackend();
  if (timing_backend.valid) {
    auto timing_stats_iter = timing_backend.timers.find(timer_name_);
    if (timing_stats_iter == std::end(timing_backend.timers)) {
      TimingStatistics new_timing_stats;
      new_timing_stats.number_of_calls = 1;
      new_timing_stats.average_time_in_milliseconds =
          measured_duration_in_milliseconds;
      new_timing_stats.min_time_in_milliseconds =
          measured_duration_in_milliseconds;
      new_timing_stats.max_time_in_milliseconds =
          measured_duration_in_milliseconds;
      timing_backend.timers.insert(
          std::make_pair(timer_name_, new_timing_stats));
      return;
    }

    // augment existing stats
    TimingStatistics& timing_stats = timing_stats_iter->second;
    timing_stats.average_time_in_milliseconds =
        (timing_stats.average_time_in_milliseconds *
             timing_stats.number_of_calls +
         measured_duration_in_milliseconds) /
        (timing_stats.number_of_calls + 1);
    timing_stats.number_of_calls += 1;
    timing_stats.max_time_in_milliseconds =
        std::max(timing_stats.max_time_in_milliseconds,
                 measured_duration_in_milliseconds);
    timing_stats.min_time_in_milliseconds =
        std::min(timing_stats.min_time_in_milliseconds,
                 measured_duration_in_milliseconds);
  }
}

TimingAggregator::TimingAggregator(absl::string_view timing_group_name,
                                   bool print_result)
    : valid_(false) {
  if (GetTimingBackend().valid) {
    // TiminAggregator already exists in this scope
    return;
  }

  valid_ = true;
  // Establish new TimingBackend for this thread/scope
  auto timing_backend = TimingBackend(timing_group_name, print_result);
  AccessTimingBackend(&timing_backend);
}

TimingAggregator::~TimingAggregator() {
  if (!valid_) {
    // Nothing to clean up or output
    return;
  }

  // Clear timing backend
  auto& timing_backend = GetTimingBackend();

  // Output values
  if (timing_backend.print_result) {
    std::stringstream ss;
    ss << timing_backend;
    ROS_INFO_STREAM(ss.str());
  }
  PublishTimingIntrospection(timing_backend);

  // Reset timing backend to invalid
  auto invalid_backend = TimingBackend();
  AccessTimingBackend(&invalid_backend);
}

}  // namespace bookbot

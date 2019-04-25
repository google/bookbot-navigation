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

#ifndef BOOKBOT_ROS_UTILITIES_INTROSPECTION_H_
#define BOOKBOT_ROS_UTILITIES_INTROSPECTION_H_

#include <absl/synchronization/mutex.h>
#include <absl/types/optional.h>
#include <ros/ros.h>

#include <map>
#include <mutex>
#include <string>

namespace bookbot {

/// \brief Introspection publishing backend that mantains a map of registered
/// publishers by topic
class IntrospectionBackend {
 public:
  IntrospectionBackend() = default;
  IntrospectionBackend(IntrospectionBackend&& other);
  IntrospectionBackend& operator=(IntrospectionBackend&& other);
  IntrospectionBackend(const ros::NodeHandle& node_handle)
      : node_handle_(node_handle) {}

  template <typename MessageType>
  void Publish(std::string topic, MessageType&& message) {
    {
      absl::ReaderMutexLock lock(&publisher_map_mutex_);
      auto publish_map_iter = publisher_map_.find(topic);
      if (publish_map_iter != std::end(publisher_map_)) {
        ros::Publisher& publisher = publish_map_iter->second;
        publisher.publish(std::forward<MessageType>(message));
        return;
      }
    }

    // Need to register new publisher (this should be only once per topic)
    absl::WriterMutexLock lock(&publisher_map_mutex_);
    // Need to recheck after aquiring write lock to ensure it wasn't just added
    auto publish_map_iter = publisher_map_.find(topic);
    if (publish_map_iter == std::end(publisher_map_)) {
      // Safe to add new publisher
      auto publisher = node_handle_.advertise<MessageType>(topic, 1);
      auto publish_map_emplace_status =
          publisher_map_.emplace(topic, publisher);
      publish_map_iter = publish_map_emplace_status.first;
    }
    ros::Publisher& publisher = publish_map_iter->second;
    publisher.publish(std::forward<MessageType>(message));
  }

 private:
  ros::NodeHandle node_handle_;
  absl::Mutex publisher_map_mutex_;
  std::map<std::string, ros::Publisher> publisher_map_;
};

/// \brief Access to introspection backend singleton, will return a nullopt if
/// not within the scope of an Introspector
absl::optional<IntrospectionBackend>& GetIntrospectionBackend();

/// \brief RAII wrapper for IntrospectionBackend synchronization. This
/// prevents calls to ros publishing when running unit tests and other
/// ros-free code. \note This should be put in the main() function of a ros
/// node
class Introspector {
 public:
  Introspector(const ros::NodeHandle& node_handle);
  ~Introspector();
};

}  // namespace bookbot

#endif  // BOOKBOT_ROS_UTILITIES_INTROSPECTION_H_

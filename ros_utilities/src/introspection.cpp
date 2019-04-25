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

#include <ros_utilities/introspection.h>

namespace bookbot {

absl::optional<IntrospectionBackend>& AccessIntrospectionBackend(
    absl::optional<IntrospectionBackend>* reset_value) {
  static absl::optional<IntrospectionBackend> introspection_backend;
  if (reset_value) {
    introspection_backend = std::move(*reset_value);
  }
  return introspection_backend;
}

absl::optional<IntrospectionBackend>& GetIntrospectionBackend() {
  return AccessIntrospectionBackend(nullptr);
}

IntrospectionBackend::IntrospectionBackend(IntrospectionBackend&& other) {
  absl::WriterMutexLock lock(&other.publisher_map_mutex_);
  std::swap(node_handle_, other.node_handle_);
  std::swap(publisher_map_, other.publisher_map_);
}

IntrospectionBackend& IntrospectionBackend::operator=(
    IntrospectionBackend&& other) {
  if (this != &other) {
    absl::WriterMutexLock lock_other(&other.publisher_map_mutex_);
    absl::WriterMutexLock lock(&publisher_map_mutex_);
    std::swap(node_handle_, other.node_handle_);
    std::swap(publisher_map_, other.publisher_map_);
  }
  return *this;
}

Introspector::Introspector(const ros::NodeHandle& node_handle) {
  // Initialize introspection
  absl::optional<IntrospectionBackend> new_backend =
      IntrospectionBackend(node_handle);
  AccessIntrospectionBackend(&new_backend);
}

Introspector::~Introspector() {
  absl::optional<IntrospectionBackend> empty_backend;
  AccessIntrospectionBackend(&empty_backend);
}

}  // namespace bookbot

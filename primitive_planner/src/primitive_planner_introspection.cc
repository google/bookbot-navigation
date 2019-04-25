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

#include <primitive_planner/primitive_planner_introspection.h>
#include <ros_utilities/introspection.h>
#include <visualization_msgs/MarkerArray.h>

namespace bookbot {

namespace {
visualization_msgs::MarkerArray ConvertPathToMarkerArray(
    const std::string& topic, const std::string& frame, const Path& path) {
  visualization_msgs::MarkerArray path_msg_array;

  visualization_msgs::Marker path_msg;
  path_msg.header.frame_id = frame;
  path_msg.header.stamp = ros::Time::now();
  path_msg.type = visualization_msgs::Marker::LINE_STRIP;
  path_msg.id = 0;
  path_msg.scale.x = 0.1;
  path_msg.pose.orientation.w = 1.0;
  for (const PathPoint& point : path) {
    geometry_msgs::Point marker_point;
    marker_point.x = point.x;
    marker_point.y = point.y;
    marker_point.z = 0.;
    path_msg.points.push_back(marker_point);
    std_msgs::ColorRGBA marker_color;
    marker_color.r = 1;
    marker_color.g = 0.5;
    marker_color.b = 0;
    marker_color.a = 1;
    path_msg.colors.push_back(marker_color);
  }
  path_msg_array.markers.push_back(path_msg);

  visualization_msgs::Marker path_points_msg;
  path_points_msg.header.frame_id = frame;
  path_points_msg.header.stamp = ros::Time::now();
  path_points_msg.type = visualization_msgs::Marker::SPHERE_LIST;
  path_points_msg.id = 1;
  path_points_msg.scale.x = 0.1;
  path_points_msg.pose.orientation.w = 1.0;
  for (const PathPoint& point : path) {
    geometry_msgs::Point marker_point;
    marker_point.x = point.x;
    marker_point.y = point.y;
    marker_point.z = 0.;
    path_points_msg.points.push_back(marker_point);
    std_msgs::ColorRGBA marker_color;
    marker_color.r = 0;
    marker_color.g = 0.5;
    marker_color.b = 1;
    marker_color.a = 1;
    path_points_msg.colors.push_back(marker_color);
  }
  path_msg_array.markers.push_back(path_points_msg);
  return path_msg_array;
}
}  // namespace

void PublishIntrospection(const std::string& topic, const std::string& frame,
                          const Path& path) {
  absl::optional<IntrospectionBackend>& backend = GetIntrospectionBackend();
  if (!backend.has_value()) {
    return;
  }

  visualization_msgs::MarkerArray path_msg_array =
      ConvertPathToMarkerArray(topic, frame, path);

  backend.value().Publish(topic, std::move(path_msg_array));
}

void PublishIntrospection(
    const std::string& topic, const std::string& frame,
    const std::deque<PathPrimitiveSearchNode>& primitive_path,
    double interpolation_resolution) {
  if (primitive_path.empty()) {
    return;
  }
  absl::optional<IntrospectionBackend>& backend = GetIntrospectionBackend();
  if (!backend.has_value()) {
    return;
  }

  Path smooth_introspection_path;
  for (auto iter = std::next(std::begin(primitive_path));
       iter != std::end(primitive_path); ++iter) {
    const PathPrimitiveSearchNode& prev_node = *std::prev(iter);
    const PathPrimitiveSearchNode& current_node = *iter;
    PathPoint prev_node_path_point = prev_node.State();
    double length =
        current_node.distance_traveled - prev_node.distance_traveled;
    for (double distance = 0; distance < length;
         distance += interpolation_resolution) {
      PathPoint interpolated_point =
          current_node.previous_primitive.evaluator_ptr
              ->InterpolateInteriorPoint(prev_node_path_point, distance);
      smooth_introspection_path.push_back(interpolated_point);
    }
  }

  visualization_msgs::MarkerArray path_msg_array =
      ConvertPathToMarkerArray(topic, frame, smooth_introspection_path);

  backend.value().Publish(topic, std::move(path_msg_array));
}

void PublishIntrospection(
    const std::string& topic, const std::string& frame,
    const std::vector<PathPrimitiveSearchNode>& explored_tree,
    double interpolation_resolution) {
  absl::optional<IntrospectionBackend>& backend = GetIntrospectionBackend();
  if (!backend.has_value()) {
    return;
  }

  std::vector<Path> explored_tree_paths;
  for (const PathPrimitiveSearchNode& node : explored_tree) {
    if (node.parent_index == -1) {
      continue;
    }
    Path explored_segment;
    const PathPrimitiveSearchNode& prev_node = explored_tree[node.parent_index];
    PathPoint prev_node_path_point = prev_node.State();
    double length = node.distance_traveled - prev_node.distance_traveled;
    for (double distance = 0; distance < length;
         distance += interpolation_resolution) {
      PathPoint interpolated_point =
          node.previous_primitive.evaluator_ptr->InterpolateInteriorPoint(
              prev_node_path_point, distance);
      explored_segment.push_back(interpolated_point);
    }
    // Add point at end of segment
    PathPoint interpolated_point =
        node.previous_primitive.evaluator_ptr->InterpolateInteriorPoint(
            prev_node_path_point, length);
    explored_segment.push_back(interpolated_point);
    explored_tree_paths.push_back(explored_segment);
  }

  visualization_msgs::Marker explored_paths_msg;
  explored_paths_msg.header.frame_id = frame;
  explored_paths_msg.header.stamp = ros::Time::now();
  explored_paths_msg.type = visualization_msgs::Marker::LINE_LIST;
  explored_paths_msg.scale.x = 0.01;
  explored_paths_msg.pose.orientation.w = 1.0;

  auto form_point_msg = [&explored_paths_msg](const PathPoint& point) {
    geometry_msgs::Point marker_point;
    marker_point.x = point.x;
    marker_point.y = point.y;
    marker_point.z = 0.;
    explored_paths_msg.points.push_back(marker_point);
    std_msgs::ColorRGBA marker_color;
    marker_color.r = 0;
    marker_color.g = 1;
    marker_color.b = 0;
    marker_color.a = 1;
    explored_paths_msg.colors.push_back(marker_color);
  };

  for (const Path& path : explored_tree_paths) {
    if (path.size() <= 1) {
      continue;
    }
    for (auto path_iter = std::next(std::begin(path));
         path_iter != std::end(path); ++path_iter) {
      // Add line segment to LINE_LIST message
      form_point_msg(*std::prev(path_iter));
      form_point_msg(*path_iter);
    }
  }

  backend.value().Publish(topic, std::move(explored_paths_msg));
}

}  // namespace bookbot

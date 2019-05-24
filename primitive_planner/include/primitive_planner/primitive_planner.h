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

#ifndef BOOKBOT_PRIMITIVE_PLANNER_H_
#define BOOKBOT_PRIMITIVE_PLANNER_H_

#include <primitive_planner/primitive.h>
#include <primitive_planner/primitive_search_node.h>
#include <signed_distance_field/signed_distance_field.h>
#include <trajectory_math/path.h>

namespace bookbot {

struct PrimitivePlanningParams {
  double robot_radius;
  double obstacle_check_front_offset;
  bool require_perception;
  int horizon;
  double primitive_length;
  double primitive_min_turn_radius;
  int num_primitive_angle_samples;
  double travel_cost_weight;
  double path_divergence_weight;
  double path_end_divergence_weight;
  double path_end_alignment_weight;
  double max_path_divergence;
  double obstacle_weight;
  double curvature_integral_weight;

  // Visited grid parameters (sparser means more agressive pruning)
  double visited_grid_spatial_resolution;
  int num_visited_grid_angle_bins;

  // Whether to output introspection information
  bool node_introspection;

  // Timeouts
  double clicked_path_timeout;
  double odometry_timeout;
  double perception_timeout;

  // General planner parameters
  double cycle_time;
  double reinitialization_distance;
};

std::vector<PathPrimitiveWithEvaluation> GeneratePrimitiveSet(
    double primitive_length, double primitive_min_turn_radius,
    int primitive_angle_samples);

Path PlanPrimitivePath(const Path& clicked_path, const PathPoint& start_point,
                       const PrimitivePlanningParams& params,
                       const std::vector<PathPrimitiveWithEvaluation>&
                           primitive_set_with_evaluation,
                       const SignedDistanceField& distance_field,
                       const SignedDistanceField& clicked_path_field);

}  // namespace bookbot

#endif  // BOOKBOT_PRIMITIVE_PLANNER_H_

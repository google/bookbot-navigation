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

#ifndef BOOKBOT_SIGNED_DISTANCE_FIELD_H_
#define BOOKBOT_SIGNED_DISTANCE_FIELD_H_

#include <nav_msgs/OccupancyGrid.h>

#include <Eigen/Core>
#include <iostream>
#include <opencv2/core.hpp>
#include <vector>

constexpr double kMaxDistance = 10;

namespace bookbot {

class SignedDistanceField {
 public:
  SignedDistanceField() = default;
  SignedDistanceField(double resolution, Eigen::Vector2d origin,
                      Eigen::MatrixXd values);

  double Evaluate(Eigen::Vector2d query_point) const;

  Eigen::Vector2d EvaluateJacobian(Eigen::Vector2d query_point) const;

  Eigen::Matrix2d EvaluateHessian(Eigen::Vector2d query_point) const;

  const Eigen::MatrixXd& GetFieldValues() const;

  cv::Mat GetCVFieldValues() const;

  Eigen::Vector2d GetOrigin() const;

  double GetResolution() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  double resolution_;
  Eigen::Vector2d origin_;
  Eigen::MatrixXd values_;
};

std::ostream& operator<<(std::ostream& os,
                         const SignedDistanceField& distance_field);

//---------------------------------------------------
// Generate Signed Distance Field from occupancy grid

SignedDistanceField GenerateSignedDistanceField(
    const cv::Mat& occupancy_grid, Eigen::Vector2d origin,
    double occupancy_grid_resolution,
    int signed_distance_field_resolution_scaling_factor,
    int obstacle_threshold);

SignedDistanceField GenerateSignedDistanceField(
    const Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic>& occupancy_grid,
    Eigen::Vector2d origin, double occupancy_grid_resolution,
    int signed_distance_field_resolution_scaling_factor,
    int obstacle_threshold);

SignedDistanceField GenerateSignedDistanceField(
    const nav_msgs::OccupancyGrid& occupancy_grid_msg,
    int signed_distance_field_resolution_scaling_factor, int obstacle_threshold,
    bool treat_unknown_as_obstacle = true);
//---------------------------------------------------

//---------------------------------------------------
// Generate Signed Distance Field from distance function

SignedDistanceField GenerateSignedDistanceField(
    const std::function<double(Eigen::Vector2d)>& distance_function, int width,
    int height, double resolution, Eigen::Vector2d origin);

//---------------------------------------------------

}  // namespace bookbot

#endif  // BOOKBOT_SIGNED_DISTANCE_FIELD_H_

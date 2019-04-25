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

#include <signed_distance_field/signed_distance_field.h>
#include <signed_distance_field/signed_distance_field_introspection.h>
#include <tf/transform_datatypes.h>
#include <trajectory_math/curve_algorithms.h>

#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace bookbot {

SignedDistanceField::SignedDistanceField(double resolution,
                                         Eigen::Vector2d origin,
                                         Eigen::MatrixXd values)
    : resolution_(resolution), origin_(origin), values_(values) {}

double SignedDistanceField::Evaluate(Eigen::Vector2d query_point) const {
  const Eigen::Vector2d relative_point = query_point - origin_;

  // Check for out of bounds
  if (relative_point[0] < 0 || relative_point[1] < 0 ||
      relative_point[0] >= (values_.cols() - 1) * resolution_ ||
      relative_point[1] >= (values_.rows() - 1) * resolution_) {
    return kMaxDistance;
  }

  const auto x_low_index = static_cast<int>(relative_point[0] / resolution_);
  const auto y_low_index = static_cast<int>(relative_point[1] / resolution_);
  const double interp_x_fraction =
      relative_point[0] / resolution_ - x_low_index;
  const double interp_y_fraction =
      relative_point[1] / resolution_ - y_low_index;

  const double x_interp_low = LinearInterpolation(
      values_(x_low_index, y_low_index), values_(x_low_index + 1, y_low_index),
      interp_x_fraction);
  const double x_interp_high = LinearInterpolation(
      values_(x_low_index, y_low_index + 1),
      values_(x_low_index + 1, y_low_index + 1), interp_x_fraction);

  return LinearInterpolation(x_interp_low, x_interp_high, interp_y_fraction);
}

Eigen::Vector2d SignedDistanceField::EvaluateJacobian(
    Eigen::Vector2d query_point) const {
  const Eigen::Vector2d relative_point = query_point - origin_;

  // check for out of bounds
  if (relative_point[0] <= 0 || relative_point[1] <= 0 ||
      relative_point[0] >= (values_.cols() - 1) * resolution_ ||
      relative_point[1] >= (values_.rows() - 1) * resolution_) {
    return Eigen::Vector2d::Zero();
  }

  const auto x_low_index = static_cast<int>(relative_point[0] / resolution_);
  const auto y_low_index = static_cast<int>(relative_point[1] / resolution_);
  const double interp_x_fraction =
      relative_point[0] / resolution_ - x_low_index;
  const double interp_y_fraction =
      relative_point[1] / resolution_ - y_low_index;

  // LinearInterpolation() -> start + interp_fraction * (end - start)

  const double x_interp_low = LinearInterpolation(
      values_(x_low_index, y_low_index), values_(x_low_index + 1, y_low_index),
      interp_x_fraction);
  const double x_interp_low_dx = (values_(x_low_index + 1, y_low_index) -
                                  values_(x_low_index, y_low_index)) /
                                 resolution_;
  const double x_interp_high = LinearInterpolation(
      values_(x_low_index, y_low_index + 1),
      values_(x_low_index + 1, y_low_index + 1), interp_x_fraction);
  const double x_interp_high_dx = (values_(x_low_index + 1, y_low_index + 1) -
                                   values_(x_low_index, y_low_index + 1)) /
                                  resolution_;

  // double interp_value =
  //     LinearInterpolation(x_interp_low, x_interp_high, interp_y_fraction);
  const double interp_value_dx = (1 - interp_y_fraction) * x_interp_low_dx +
                                 interp_y_fraction * x_interp_high_dx;
  const double interp_value_dy = (x_interp_high - x_interp_low) / resolution_;
  return Eigen::Vector2d(interp_value_dx, interp_value_dy);
}

Eigen::Matrix2d SignedDistanceField::EvaluateHessian(
    Eigen::Vector2d query_point) const {
  // TODO
}

const Eigen::MatrixXd& SignedDistanceField::GetFieldValues() const {
  return values_;
}

cv::Mat SignedDistanceField::GetCVFieldValues() const {
  cv::Mat opencv_values;
  cv::eigen2cv(values_, opencv_values);
  return opencv_values;
}

Eigen::Vector2d SignedDistanceField::GetOrigin() const { return origin_; }

double SignedDistanceField::GetResolution() const { return resolution_; }

std::ostream& operator<<(std::ostream& os,
                         const SignedDistanceField& distance_field) {
  os << distance_field.GetFieldValues();
  return os;
}

SignedDistanceField GenerateSignedDistanceField(
    const cv::Mat& occupancy_grid, Eigen::Vector2d origin,
    double occupancy_grid_resolution,
    int signed_distance_field_resolution_scaling_factor,
    int obstacle_threshold) {
  // Example initial occupancy_grid:
  // -1 0 0 0 100 70
  //  0 0 0 0  80 60
  //  0 0 0 0  0  0
  // -1 0 0 0  0  0
  // -1 0 0 0  0  0

  // Threshold the occupancy grid to get an inverted binary obstacle grid where
  // free space is a 1 and obstacles are a 0. Then erode by one pixel on the top
  // and right to be conservative. The signed_distance_field treats cells
  // as points at their corner so to ensure the signed distance treats the
  // entire cell area as an obstacle we need to bloat the obstacle region by one
  // cell on the top and left. We then apply an opencv distance transform.
  // This yields distances for the free space:
  // 1 1 1 1 0 0        1 1 1 0 0 0        3   2   1   0   0   0
  // 1 1 1 1 0 0        1 1 1 0 0 0        3   2   1   0   0   0
  // 1 1 1 1 1 1  ==>   1 1 1 1 1 1  ==>   3.2 2.2 1.4 1   1   1
  // 1 1 1 1 1 1        1 1 1 1 1 1        3.6 2.8 2.2 2   2   2
  cv::Mat free_space_grid;
  threshold(occupancy_grid, free_space_grid, obstacle_threshold, 1,
            cv::THRESH_BINARY_INV);

  cv::Mat biased_erode_kernel =
      (cv::Mat_<uchar>(3, 3) << 1, 1, 0, 1, 1, 0, 0, 0, 0);
  cv::erode(free_space_grid, free_space_grid, biased_erode_kernel);
  cv::distanceTransform(free_space_grid, free_space_grid,
                        cv::DistanceTypes::DIST_L2,
                        cv::DistanceTransformMasks::DIST_MASK_5);

  // Threshold the occupancy grid to get an normal binary obstacle grid where
  // free space is a 0 and obstacles are a 1 and apply a similar erosion as
  // beore. Then apply an opencv distance transform. This yields distances for
  // the in-obstacle space:
  // 0 0 0 0 1 1        0 0 0 0 1 1
  // 0 0 0 0 1 1        0 0 0 0 0 0
  // 0 0 0 0 0 0  ==>   0 0 0 0 0 0
  // 0 0 0 0 0 0        0 0 0 0 0 0
  cv::Mat in_obstacle_grid;
  threshold(occupancy_grid, in_obstacle_grid, obstacle_threshold, 1,
            cv::THRESH_BINARY);

  cv::erode(in_obstacle_grid, in_obstacle_grid, biased_erode_kernel);
  cv::distanceTransform(in_obstacle_grid, in_obstacle_grid,
                        cv::DistanceTypes::DIST_L2,
                        cv::DistanceTransformMasks::DIST_MASK_5);

  // Subtract the in-obstacle distances from the free space distances:
  // 3   2   1   0   0   0     0 0 0 0 1 1       3   2   1    0  -1  -1
  // 3   2   1   0   0   0     0 0 0 0 0 0       3   2   1    0   0   0
  // 3.2 2.2 1.4 1   1   1  -  0 0 0 0 0 0  ==>  3.2 2.2 1.4  1   1   1
  // 3.6 2.8 2.2 2   2   2     0 0 0 0 0 0       3.6 2.8 2.2  2   2   2
  cv::Mat combined_grid = (free_space_grid - in_obstacle_grid);

  // Resize by an integer factor if a sparser distance field is desired.
  double scaling_factor = 1.0 / signed_distance_field_resolution_scaling_factor;
  cv::resize(combined_grid, combined_grid, cv::Size(), scaling_factor,
             scaling_factor, cv::InterpolationFlags::INTER_NEAREST);

  Eigen::MatrixXd distance_field;
  cv::cv2eigen(combined_grid, distance_field);

  // Scale distance field to normalize distances
  distance_field *= occupancy_grid_resolution;

  return SignedDistanceField(signed_distance_field_resolution_scaling_factor *
                                 occupancy_grid_resolution,
                             origin, distance_field);
}

SignedDistanceField GenerateSignedDistanceField(
    const Eigen::Matrix<int16_t, Eigen::Dynamic, Eigen::Dynamic>&
        occupancy_grid,
    Eigen::Vector2d origin, double occupancy_grid_resolution,
    int signed_distance_field_resolution_scaling_factor,
    int obstacle_threshold) {
  cv::Mat opencv_grid;
  cv::eigen2cv(occupancy_grid, opencv_grid);
  return GenerateSignedDistanceField(
      opencv_grid, origin, occupancy_grid_resolution,
      signed_distance_field_resolution_scaling_factor, obstacle_threshold);
}

SignedDistanceField GenerateSignedDistanceField(
    const nav_msgs::OccupancyGrid& occupancy_grid_msg,
    int signed_distance_field_resolution_scaling_factor,
    int obstacle_threshold) {
  assert(tf::getYaw(occupancy_grid_msg.info.origin.orientation) == 0 &&
         "Occupancy grid must be aligned with reference frame to creat signed "
         "distance field");
  auto origin = Eigen::Vector2d(occupancy_grid_msg.info.origin.position.x,
                                occupancy_grid_msg.info.origin.position.y);
  auto occupancy_grid_resolution = occupancy_grid_msg.info.resolution;
  auto opencv_grid = cv::Mat(occupancy_grid_msg.info.width,
                             occupancy_grid_msg.info.height, CV_8U);
  auto msg_data_iter = occupancy_grid_msg.data.begin();
  for (int row = 0; row < occupancy_grid_msg.info.height; ++row) {
    for (int column = 0; column < occupancy_grid_msg.info.width; ++column) {
      opencv_grid.at<uint8_t>(column, row) = *msg_data_iter++;
    }
  }
  return GenerateSignedDistanceField(
      opencv_grid, origin, occupancy_grid_resolution,
      signed_distance_field_resolution_scaling_factor, obstacle_threshold);
}

SignedDistanceField GenerateSignedDistanceField(
    const std::function<double(Eigen::Vector2d)>& distance_function, int width,
    int height, double resolution, Eigen::Vector2d origin) {
  auto values = Eigen::MatrixXd(width, height);
  for (auto i = 0; i < width; ++i) {
    for (auto j = 0; j < height; ++j) {
      auto point = Eigen::Vector2d(origin[0] + i * resolution,
                                   origin[1] + j * resolution);
      values(i, j) = distance_function(point);
    }
  }

  return SignedDistanceField(resolution, origin, values);
}

}  // namespace bookbot

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

#include <primitive_planner/primitive.h>
#include <trajectory_math/cubic_spiral.h>

namespace bookbot {

// Set primitive to invalid if default constructed
PathPrimitive::PathPrimitive() : primitive_type(PathPrimitiveType::kInvalid) {}

class StandStillPrimitiveEvaluator : public PathPrimitiveEvaluationBase {
 public:
  PathPoint InterpolateInteriorPoint(
      const PathPoint& start_point,
      double interpolation_distance_along_primitive) const override {
    return start_point;
  }

  PathPrimitive GeneratePrimitive() const override {
    PathPrimitive primitive;
    primitive.delta_yaw = 0;
    primitive.x_rel = 0;
    primitive.y_rel = 0;
    primitive.length = 0;
    primitive.primitive_type = PathPrimitiveType::kStandStill;
    primitive.squared_curvature_integral = 0;
    primitive.evaluator_ptr = this;
    return primitive;
  }

  std::string GetDescription() const override {
    return "[Stand Still Primitive]";
  }
};

PathPrimitiveWithEvaluation GenerateStandStillPrimitive() {
  return PathPrimitiveWithEvaluation(
      std::unique_ptr<PathPrimitiveEvaluationBase>(
          new StandStillPrimitiveEvaluator()));
}

class TurnInPlacePrimitiveEvaluator : public PathPrimitiveEvaluationBase {
 public:
  TurnInPlacePrimitiveEvaluator(double delta_yaw) : delta_yaw_(delta_yaw) {}

  PathPoint InterpolateInteriorPoint(
      const PathPoint& start_point,
      double interpolation_distance_along_primitive) const override {
    if (interpolation_distance_along_primitive > 0) {
      auto rotated_state = start_point;
      rotated_state.yaw = start_point.yaw + delta_yaw_;
      return rotated_state;
    }
    return start_point;
  }

  PathPrimitive GeneratePrimitive() const override {
    PathPrimitive primitive;
    primitive.delta_yaw = delta_yaw_;
    primitive.x_rel = 0;
    primitive.y_rel = 0;
    primitive.length = 0;
    primitive.primitive_type = PathPrimitiveType::kTurnInPlace;
    primitive.squared_curvature_integral =
        kTurnInPlaceCurvaturePenalty * std::abs(delta_yaw_);
    primitive.evaluator_ptr = this;
    return primitive;
  }

  std::string GetDescription() const override {
    std::stringstream ss;
    ss << "[Turn in place primitive - delta_yaw:" << delta_yaw_ << "]";
    return ss.str();
  }

 private:
  double delta_yaw_;
};

PathPrimitiveWithEvaluation GenerateTurnInPlacePrimitive(double delta_yaw) {
  return PathPrimitiveWithEvaluation(
      std::unique_ptr<PathPrimitiveEvaluationBase>(
          new TurnInPlacePrimitiveEvaluator(delta_yaw)));
}

class CubicSpiralPrimitiveEvaluator : public PathPrimitiveEvaluationBase {
 public:
  CubicSpiralPrimitiveEvaluator(double length, double delta_yaw)
      : length_(length), delta_yaw_(delta_yaw) {}

  PathPoint InterpolateInteriorPoint(
      const PathPoint& start_point,
      double interpolation_distance_along_primitive) const override {
    return InterpolateCubicSpiralSegmentByDistanceAlongPath(
        start_point, length_, start_point.yaw + delta_yaw_, 0.,
        interpolation_distance_along_primitive);
  }

  PathPrimitive GeneratePrimitive() const override {
    PathPoint start_point;
    start_point.x = 0;
    start_point.y = 0;
    start_point.yaw = 0;
    start_point.curvature = 0;
    start_point.distance_along_path = 0;
    const PathPoint end_point =
        InterpolateCubicSpiralSegmentByDistanceAlongPath(
            start_point, length_, delta_yaw_, 0., length_);
    PathPrimitive primitive;
    primitive.primitive_type = PathPrimitiveType::kCubicSpiral;
    primitive.x_rel = end_point.x;
    primitive.y_rel = end_point.y;
    primitive.delta_yaw = end_point.yaw;
    primitive.length = end_point.distance_along_path;
    primitive.primitive_type = PathPrimitiveType::kCubicSpiral;
    primitive.squared_curvature_integral =
        EvaluateCubicSpiralSegmentSquaredCurvatureIntegral(start_point, length_,
                                                           delta_yaw_, 0.);
    primitive.evaluator_ptr = this;

    return primitive;
  }

  std::string GetDescription() const override {
    std::stringstream ss;
    ss << "[Cubic Spiral Primitive - length:" << length_
       << " delta_yaw:" << delta_yaw_ << "]";
    return ss.str();
  }

 private:
  double length_;
  double delta_yaw_;
};

PathPrimitiveWithEvaluation GenerateCubicSpiralPrimitive(double length,
                                                         double delta_yaw) {
  return PathPrimitiveWithEvaluation(
      std::unique_ptr<PathPrimitiveEvaluationBase>(
          new CubicSpiralPrimitiveEvaluator(length, delta_yaw)));
}

PathPrimitiveWithEvaluation::PathPrimitiveWithEvaluation(
    std::unique_ptr<PathPrimitiveEvaluationBase> primitive_evaluator)
    : primitive_evaluator_(std::move(primitive_evaluator)) {}

PathPrimitive PathPrimitiveWithEvaluation::GetPrimitive() const {
  return primitive_evaluator_->GeneratePrimitive();
}

}  // namespace bookbot

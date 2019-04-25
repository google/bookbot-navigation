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

#ifndef BOOKBOT_BOOKBOT_BASE_SIMULATOR_ROBOT_MOTION_MODEL_H_
#define BOOKBOT_BOOKBOT_BASE_SIMULATOR_ROBOT_MOTION_MODEL_H_
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace bookbot {
namespace simulator {

struct ControlCommand {
  ControlCommand() : forward_velocity(0), angular_velocity(0) {}
  double forward_velocity;
  double angular_velocity;
};

struct RobotSimulatorState {
  RobotSimulatorState()
      : x(0), y(0), yaw(0), forward_velocity(0), angular_velocity(0) {}
  double x;
  double y;
  double yaw;
  double forward_velocity;
  double angular_velocity;
};

Eigen::Isometry3d ExtractTransform(const RobotSimulatorState& sim_state);

/// \brief Simple unicycle dynamics model functor to serve as a default
struct UnicycleModel {
  RobotSimulatorState operator()(const RobotSimulatorState& initial_state,
                                 const ControlCommand& command,
                                 double delta_time);
};

template <typename DynamicsModelFunctor = UnicycleModel>
class RobotModel {
 public:
  explicit RobotModel(
      const RobotSimulatorState& initial_robot_state,
      DynamicsModelFunctor dynamics_model = DynamicsModelFunctor())
      : current_state_(initial_robot_state),
        dynamics_model_(std::move(dynamics_model)) {}

  void ApplyControl(const ControlCommand& command) {
    last_control_command_ = command;
  }

  void Step(double delta_time) {
    current_state_ =
        dynamics_model_(current_state_, last_control_command_, delta_time);
  }

  const RobotSimulatorState& GetCurrentState() const { return current_state_; }

 private:
  ControlCommand last_control_command_;
  RobotSimulatorState current_state_;
  DynamicsModelFunctor dynamics_model_;
};

}  // namespace simulator
}  // namespace bookbot

#endif  // BOOKBOT_BOOKBOT_BASE_SIMULATOR_ROBOT_MOTION_MODEL_H_

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
#include <bookbot_base_simulator/robot_motion_model.h>

namespace bookbot {
namespace simulator {

Eigen::Isometry3d ExtractTransform(const RobotSimulatorState& sim_state) {
  Eigen::Isometry3d transform;
  transform = Eigen::Translation3d(sim_state.x, sim_state.y, 0.) *
              Eigen::AngleAxisd(sim_state.yaw, Eigen::Vector3d::UnitZ());
  return transform;
}

RobotSimulatorState UnicycleModel::operator()(
    const RobotSimulatorState& initial_state, const ControlCommand& command,
    double delta_time) {
  RobotSimulatorState output_state;
  output_state.forward_velocity = command.forward_velocity;
  output_state.angular_velocity = command.angular_velocity;
  output_state.yaw = initial_state.yaw + command.angular_velocity * delta_time;
  output_state.x = initial_state.x + command.forward_velocity *
                                         std::cos(initial_state.yaw) *
                                         delta_time;
  output_state.y = initial_state.y + command.forward_velocity *
                                         std::sin(initial_state.yaw) *
                                         delta_time;
  return output_state;
}

}  // namespace simulator
}  // namespace bookbot

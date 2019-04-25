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

#include <absl/time/clock.h>
#include <bookbot_base_simulator/bookbot_base_simulator.h>

namespace bookbot {
namespace simulator {

SimTime::SimTime() { current_time_ = absl::Seconds(0); }

SimTime::SimTime(double seconds) { current_time_ = absl::Seconds(seconds); }

void SimTime::Step(double delta_time) {
  current_time_ += absl::Seconds(delta_time);
}

double SimTime::GetTimeAsDouble() const {
  return absl::ToDoubleSeconds(current_time_);
}

BaseSimulator::BaseSimulator()
    : robot_model_(RobotSimulatorState()),
      sim_time_(absl::ToDoubleSeconds(absl::Now() - absl::UnixEpoch())) {}

void BaseSimulator::Initialize(const RobotSimulatorState& initial_robot_state,
                               SimTime initial_sim_time) {
  robot_model_ = SimRobotModel(initial_robot_state);
  sim_time_ = initial_sim_time;
}

void BaseSimulator::SetControlCommand(const ControlCommand& command) {
  last_control_command_ = command;
}

SimulatorWorldState BaseSimulator::Step(double delta_time) {
  Step(delta_time, last_control_command_);
}

SimulatorWorldState BaseSimulator::Step(double delta_time,
                                        ControlCommand command) {
  robot_model_.ApplyControl(command);
  robot_model_.Step(delta_time);
  sim_time_.Step(delta_time);
  return GetCurrentSimState();
}

SimulatorWorldState BaseSimulator::GetCurrentSimState() const {
  return {robot_model_.GetCurrentState(), sim_time_};
}

}  // namespace simulator
}  // namespace bookbot

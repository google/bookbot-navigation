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

#ifndef BOOKBOT_SIDEWALKER_BASE_SIMULATOR_H_
#define BOOKBOT_SIDEWALKER_BASE_SIMULATOR_H_

#include <absl/time/time.h>
#include <bookbot_base_simulator/robot_motion_model.h>

namespace bookbot {
namespace simulator {

class SimTime {
 public:
  SimTime();
  explicit SimTime(double seconds);
  void Step(double delta_time);
  double GetTimeAsDouble() const;

 private:
  absl::Duration current_time_;
};

struct SimulatorWorldState {
  RobotSimulatorState robot_state;
  SimTime sim_time;
};

class BaseSimulator {
  using SimRobotModel = RobotModel<>;

 public:
  BaseSimulator();
  void Initialize(const RobotSimulatorState& initial_robot_state,
                  SimTime initial_sim_time);
  void SetControlCommand(const ControlCommand& command);
  SimulatorWorldState Step(double delta_time);
  SimulatorWorldState Step(double delta_time, ControlCommand command);
  SimulatorWorldState GetCurrentSimState() const;

 private:
  ControlCommand last_control_command_;
  SimRobotModel robot_model_;
  SimTime sim_time_;
};

}  // namespace simulator
}  // namespace bookbot

#endif  // BOOKBOT_SIDEWALKER_BASE_SIMULATOR_H_

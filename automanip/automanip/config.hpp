/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file config.hpp
 * @brief Process configuration. The arm block mirrors chassis in autodriver.
 */

#ifndef AUTOMANIP_CONFIG_HPP_
#define AUTOMANIP_CONFIG_HPP_

#include <string>

#include "arm/plant.hpp"

namespace automanip {

struct Config {
  std::string node_name = "automanip";

  struct Arm {
    bool enable = true;
    std::string id = "arm/manipulator";
    /** @brief ArmBackendRegistry key: stub / osc2 (alias ocs2). */
    std::string backend = "osc2";
    /** @brief hold / home / track, applied after the driver starts. */
    std::string initial_mode = "hold";
    int control_period_ms = 20;
    /** @brief No new pose for this long while tracking → hold. 0 disables. */
    int watchdog_ms = 500;
    int capability_period_ticks = 50;
    std::string pose_channel = "/arm/target_pose";
    std::string joint_command_channel = "/arm/joint_command";
    std::string mode_channel = "/arm/mode";
    std::string joint_state_channel = "/arm/joint_states";
    std::string ee_pose_channel = "/arm/ee_pose";
    std::string mode_state_channel = "/arm/mode_state";
    std::string capability_channel = "/arm/capability";
    std::string event_channel = "/arm/event";
    arm::ArmPlantOptions plant;
  } arm;
};

}  // namespace automanip

#endif  // AUTOMANIP_CONFIG_HPP_

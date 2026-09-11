/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file capability.hpp
 * @brief Static chassis capability profile for downstream (nav / task).
 */

#ifndef AUTODRIVER_CHASSIS_CAPABILITY_HPP_
#define AUTODRIVER_CHASSIS_CAPABILITY_HPP_

#include <string>
#include <vector>

#include "chassis/locomotion_model.hpp"

namespace autodriver {
namespace chassis {

/**
 * @brief One-shot / latched capability advertisement (not per-tick state).
 *
 * Downstream should key off flags and locomotion, not product nicknames.
 */
struct CapabilityProfile {
  std::string chassis_id;           ///< e.g. chassis/base
  std::string backend;              ///< Registry key
  LocomotionModel locomotion;
  bool has_dock = false;            ///< Hardware dock / charge contact
  bool has_tool_channel = false;    ///< Job tools (brush, blade, …) separate
  bool has_joint_bypass = false;    ///< High-rate joints on a side channel
  bool require_arm = false;         ///< Must arm before accepting twist
  std::vector<std::string> tools;   ///< Optional tool names from YAML
};

/**
 * @brief Compact JSON for std_msgs/String capability channel (no extra deps).
 */
std::string CapabilityProfileToJson(const CapabilityProfile& profile);

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_CAPABILITY_HPP_

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
 * @file dualsense_profile.hpp
 * @brief Sony DualSense (PS5) axis/button presets for differential chassis teleop.
 *
 * Mapping assumes Linux hid-playstation + joydev (USB or Bluetooth):
 *   axes[1]  left stick Y   → linear.x (inverted: up = forward)
 *   axes[2]  L2             — rests at -1, not a stick
 *   axes[3]  right stick X  → angular.z
 *   max_linear_acc / max_angular_acc of 0 follow the stick with no ramp
 *
 * If jstest shows different indices (USB vs BT / kernel version), override in YAML.
 */

#ifndef AUTODRIVER_JOY_DUALSENSE_PROFILE_HPP_
#define AUTODRIVER_JOY_DUALSENSE_PROFILE_HPP_

#include <string>

#include "autodriver/config.hpp"

namespace autodriver {
namespace joy {

/**
 * @brief Apply known controller profile defaults onto @p options.
 * @param[in] profile Profile name: dualsense / ps5 / generic (no-op).
 * @param[in,out] options Joy config to update (axis/button/deadzone/frame).
 * @return true if a known profile was applied; false if unknown/empty/generic.
 *
 * Does not change enable, device, or channel names. YAML keys written after
 * the profile still override these defaults.
 */
inline bool ApplyJoyProfile(const std::string& profile, Config::Joy* options) {
  if (options == nullptr || profile.empty()) {
    return false;
  }
  std::string name = profile;
  for (char& c : name) {
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
  }
  if (name == "generic" || name == "none") {
    return false;
  }
  if (name == "dualsense" || name == "ps5" || name == "sony_dualsense") {
    // Left stick Y forward, right stick X yaw. Bluetooth connect if js* is absent.
    options->frame_id = "dualsense";
    options->linear_axis = 1;
    options->angular_axis = 3;
    options->invert_linear = true;
    options->invert_angular = true;
    options->deadzone = 0.05f;
    options->max_linear = 1.5;
    options->max_angular = 1.5;
    options->max_linear_acc = 0.0;
    options->max_angular_acc = 0.0;
    options->enable_button = 4;  // L1, unused unless require_enable
    options->require_enable = false;
    options->bluetooth_connect = true;
    return true;
  }
  return false;
}

}  // namespace joy
}  // namespace autodriver

#endif  // AUTODRIVER_JOY_DUALSENSE_PROFILE_HPP_

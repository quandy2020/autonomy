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
 * Mapping assumes Linux hid-playstation / standard joystick interface:
 *   axes[0]  left stick X   → angular.z
 *   axes[1]  left stick Y   → linear.x (inverted: up = forward)
 *   buttons[4] L1           → enable (hold to move)
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
 * Does not change enable, device, channels, scales, or require_enable so that
 * YAML can set those independently. Call before applying explicit axis overrides
 * if you want YAML fields to win; JoyTeleop calls this then keeps YAML values.
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
    // Left-stick arcade: Y forward, X yaw. L1 = enable.
    options->frame_id = "dualsense";
    options->linear_axis = 1;
    options->angular_axis = 0;
    options->invert_linear = true;
    options->invert_angular = false;
    options->deadzone = 0.08f;
    options->enable_button = 4;  // L1
    options->require_enable = true;
    return true;
  }
  return false;
}

}  // namespace joy
}  // namespace autodriver

#endif  // AUTODRIVER_JOY_DUALSENSE_PROFILE_HPP_

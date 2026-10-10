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
 * @file mode.hpp
 * @brief Arm operational mode names shared by drivers and the manager.
 */

#ifndef AUTOMANIP_ARM_MODE_HPP_
#define AUTOMANIP_ARM_MODE_HPP_

#include <string>

namespace automanip {
namespace arm {

enum class ArmMode {
  kHold,   ///< Keep the current joints.
  kHome,   ///< Track the configured home posture.
  kTrack,  ///< Track the end-effector pose.
  kEstop,  ///< Motion disabled until clear_estop.
  kFault,  ///< Latched fault until clear_fault.
};

inline const char* ArmModeName(ArmMode mode) {
  switch (mode) {
    case ArmMode::kHold:
      return "hold";
    case ArmMode::kHome:
      return "home";
    case ArmMode::kTrack:
      return "track";
    case ArmMode::kEstop:
      return "estop";
    case ArmMode::kFault:
      return "fault";
  }
  return "hold";
}

/**
 * @brief Parse a mode command.
 * @return false when @p text is not a known command.
 */
inline bool ParseArmModeCommand(const std::string& text, ArmMode* mode) {
  if (mode == nullptr) {
    return false;
  }
  if (text == "hold" || text == "idle") {
    *mode = ArmMode::kHold;
    return true;
  }
  if (text == "home") {
    *mode = ArmMode::kHome;
    return true;
  }
  if (text == "track") {
    *mode = ArmMode::kTrack;
    return true;
  }
  if (text == "estop" || text == "e_stop" || text == "stop") {
    *mode = ArmMode::kEstop;
    return true;
  }
  if (text == "clear_estop" || text == "clear_fault") {
    *mode = ArmMode::kHold;
    return true;
  }
  return false;
}

}  // namespace arm
}  // namespace automanip

#endif  // AUTOMANIP_ARM_MODE_HPP_

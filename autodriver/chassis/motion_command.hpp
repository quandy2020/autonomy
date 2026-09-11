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
 * @file motion_command.hpp
 * @brief Unified motion command: Twist + optional locomotion intent.
 *
 * Wire cmd_vel stays geometry_msgs/TwistStamped; mode/intent use a sibling
 * std_msgs/String channel so legacy publishers keep working.
 */

#ifndef AUTODRIVER_CHASSIS_MOTION_COMMAND_HPP_
#define AUTODRIVER_CHASSIS_MOTION_COMMAND_HPP_

#include "chassis/operational_mode.hpp"
#include "chassis/types.hpp"

namespace autodriver {
namespace chassis {

/**
 * @brief In-process motion request after Manager clamps / gates.
 */
struct MotionCommand {
  ChassisCommand twist;  ///< Body twist (zero = soft stop).
  LocomotionIntent intent = LocomotionIntent::kUnspecified;
  bool has_intent = false;  ///< When true, driver should ApplyLocomotionIntent.
};

/**
 * @brief True when linear or angular magnitude is meaningfully non-zero.
 */
inline bool IsNonZeroTwist(const ChassisCommand& cmd) {
  if (!cmd.has_twist()) {
    return false;
  }
  const auto& t = cmd.twist();
  constexpr double kEps = 1e-6;
  const double lx = t.has_linear() ? t.linear().x() : 0.0;
  const double ly = t.has_linear() ? t.linear().y() : 0.0;
  const double lz = t.has_linear() ? t.linear().z() : 0.0;
  const double ax = t.has_angular() ? t.angular().x() : 0.0;
  const double ay = t.has_angular() ? t.angular().y() : 0.0;
  const double az = t.has_angular() ? t.angular().z() : 0.0;
  return (lx * lx + ly * ly + lz * lz + ax * ax + ay * ay + az * az) > kEps * kEps;
}

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_MOTION_COMMAND_HPP_

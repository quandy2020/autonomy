/*
 * Copyright 2026 Autodriver contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file
 * @brief Chassis identity and motion types (no autonomy/vehicle dependency).
 */

#ifndef AUTODRIVER_CHASSIS_TYPES_HPP_
#define AUTODRIVER_CHASSIS_TYPES_HPP_

#include <cstdint>
#include <string>

namespace autodriver {
namespace chassis {

/** Stable chassis instance id, e.g. "chassis/base". */
using ChassisId = std::string;

/** Platform kinematics class (vendor maps SDK → this). */
enum class ChassisKind {
  kUnknown = 0,
  kDifferential = 1,
  kAckermann = 2,
  kOmni = 3,
  kMecanum = 4,
};

/**
 * @brief Velocity / actuator command sent to the robot body.
 *
 * Units: m/s, rad/s. Vendors translate to motor / CAN / SDK calls.
 * Deliberately not autonomy::vehicle::KinematicsControlCommand.
 */
struct ChassisCommand {
  std::uint64_t stamp_ns = 0;
  double linear_x = 0.0;
  double linear_y = 0.0;
  double angular_z = 0.0;
  bool emergency_stop = false;
};

/**
 * @brief Runtime state reported by the robot body.
 *
 * Pose is typically odom←base; vendors fill what their SDK provides.
 */
struct ChassisState {
  std::uint64_t stamp_ns = 0;
  ChassisKind kind = ChassisKind::kUnknown;

  double pose_x = 0.0;
  double pose_y = 0.0;
  double pose_yaw = 0.0;

  double linear_x = 0.0;
  double linear_y = 0.0;
  double angular_z = 0.0;

  bool emergency_stop_active = false;
  bool fault_active = false;
  std::int32_t fault_code = 0;

  double battery_soc = -1.0;
  double battery_voltage = 0.0;
};

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_TYPES_HPP_

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
 * @file locomotion_model.hpp
 * @brief Kinematic model declared to planners (not robot product type).
 */

#ifndef AUTODRIVER_CHASSIS_LOCOMOTION_MODEL_HPP_
#define AUTODRIVER_CHASSIS_LOCOMOTION_MODEL_HPP_

#include <string>

namespace autodriver {
namespace chassis {

/**
 * @brief Planar / body locomotion class used by CapabilityProfile.
 *
 * Downstream branches on this (and flags), not on "mower" / "AGV" labels.
 */
enum class LocomotionType {
  kUnknown = 0,
  kDifferential,  ///< Diff-drive / tracked (vx, wz); no sustained vy.
  kOmnidirectional,  ///< Holonomic planar (vx, vy, wz).
  kAckermann,     ///< Car-like; min turning radius applies.
  kLegged,        ///< Quadruped / biped gait maps twist → locomotion.
  kWheelLegged,   ///< Wheel-leg hybrid (mode selects wheel vs walk).
  kHumanoid,      ///< Whole-body; nav still often consumes twist.
};

/**
 * @brief Static kinematics limits and flags (from YAML, baked into capability).
 */
struct LocomotionModel {
  LocomotionType type = LocomotionType::kDifferential;
  /** @brief Soft max |vx|,|vy| (m/s); 0 = unlimited (Manager may still clamp). */
  double max_linear_speed = 0.0;
  /** @brief Soft max |wz| (rad/s); 0 = unlimited. */
  double max_angular_speed = 0.0;
  /** @brief Soft max |ax| (m/s^2); 0 = not enforced here. */
  double max_linear_accel = 0.0;
  /** @brief Min turning radius (m) for Ackermann; 0 = N/A. */
  double min_turning_radius = 0.0;
  /** @brief True when sustained body-frame vy is supported. */
  bool supports_lateral = false;
  /** @brief True when in-place yaw is allowed. */
  bool supports_inplace_turn = true;
};

/**
 * @brief Parse locomotion type from YAML / CLI string (case-insensitive aliases).
 * @param[in] text e.g. differential, omni, ackermann, quadruped, wheel_leg, humanoid.
 * @return Parsed type, or kUnknown when unrecognized.
 */
LocomotionType ParseLocomotionType(const std::string& text);

/**
 * @brief Stable lowercase name for JSON / mode state.
 */
const char* LocomotionTypeToString(LocomotionType type);

/**
 * @brief Fill default flags for @p type (lateral / inplace) when YAML omits them.
 */
void ApplyLocomotionTypeDefaults(LocomotionModel* model);

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_LOCOMOTION_MODEL_HPP_

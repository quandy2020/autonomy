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
 * @file safety_gate.hpp
 * @brief Shared safety path: mode, E-stop latch, watchdog, kinematics clamp.
 */

#ifndef AUTODRIVER_CHASSIS_SAFETY_GATE_HPP_
#define AUTODRIVER_CHASSIS_SAFETY_GATE_HPP_

#include <cstdint>

#include "chassis/locomotion_model.hpp"
#include "chassis/operational_mode.hpp"
#include "chassis/types.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace chassis {

/**
 * @struct autodriver::chassis::SafetyLimits
 * @brief Limits used when clamping twist before the vendor driver.
 */
struct SafetyLimits {
  LocomotionModel model;   ///< Kinematics + max speeds from Capability.
  bool require_arm = false;  ///< Idle rejects twist when true.
  int watchdog_ms = 200;     ///< cmd_vel timeout; 0 disables.
};

/**
 * @class autodriver::chassis::SafetyGate
 * @brief Chassis-wide safety gate (morphology-agnostic).
 *
 * Always runs before ChassisDriver::ApplyVelocityCommand so differential,
 * legged, and humanoid backends share one safety path.
 */
class SafetyGate {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(SafetyGate)

  /**
   * @brief Disable copy construction and copy assignment.
   */
  DISALLOW_COPY_AND_ASSIGN(SafetyGate)

  /**
   * @brief Construct with kinematic / watchdog limits from YAML.
   * @param[in] limits Clamps, require_arm, and watchdog period.
   */
  explicit SafetyGate(SafetyLimits limits) : limits_(std::move(limits)) {}

  /**
   * @brief Replace limits (e.g. after config reload).
   * @param[in] limits New clamp / watchdog settings.
   */
  void SetLimits(SafetyLimits limits) { limits_ = std::move(limits); }

  /**
   * @brief Access the active limits.
   */
  const SafetyLimits& limits() const { return limits_; }

  /**
   * @brief Clamp twist to model max_* and zero lateral when unsupported.
   * @param[in] in Raw TwistStamped from Autolink.
   * @return Clamped copy (header preserved).
   */
  ChassisCommand ClampVelocity(const ChassisCommand& in) const;

  /**
   * @brief Whether the mode FSM allows forwarding motion.
   * @param[in] modes OperationalModeController owned by ChassisManager.
   * @return true when twist may be sent to the driver.
   */
  bool AllowsMotion(const OperationalModeController& modes) const {
    return modes.AllowsMotion(limits_.require_arm);
  }

  /**
   * @brief Soft-stop predicate when cmd_vel watchdog expires.
   * @param[in] now_ns Steady-clock now (nanoseconds).
   * @param[in] last_cmd_ns Steady-clock of last accepted cmd_vel.
   * @return true when a zero twist should be applied now.
   */
  bool WatchdogExpired(std::uint64_t now_ns, std::uint64_t last_cmd_ns) const;

private:
  SafetyLimits limits_;  ///< Active clamp / arm / watchdog settings.
};

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_SAFETY_GATE_HPP_

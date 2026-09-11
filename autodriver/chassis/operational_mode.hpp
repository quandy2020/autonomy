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
 * @file operational_mode.hpp
 * @brief Shared chassis mode FSM (Idle→Armed→Moving→Docking / Fault / EStop).
 */

#ifndef AUTODRIVER_CHASSIS_OPERATIONAL_MODE_HPP_
#define AUTODRIVER_CHASSIS_OPERATIONAL_MODE_HPP_

#include <mutex>
#include <string>

#include "autolink/common/macros.hpp"

namespace autodriver {
namespace chassis {

/**
 * @enum autodriver::chassis::OperationalMode
 * @brief Operational mode shared by differential, legged, humanoid, …
 *
 * Gait / stand / wheel are locomotion *intents* layered on Armed/Moving, not
 * separate top-level modes.
 */
enum class OperationalMode {
  kIdle = 0,     ///< Powered, motion rejected unless require_arm=false
  kArmed,        ///< Accepts twist
  kMoving,       ///< Non-zero twist recently applied
  kDocking,      ///< Dock maneuver (twist may be internal)
  kFault,        ///< Latched fault; needs clear_fault
  kEStop         ///< Emergency stop latched; needs clear_estop
};

/**
 * @enum autodriver::chassis::LocomotionIntent
 * @brief Optional locomotion intent for legged / wheel-leg / humanoid backends.
 */
enum class LocomotionIntent {
  kUnspecified = 0,  ///< No intent change
  kStand,            ///< Hold / stand
  kWalk,             ///< Walking gait
  kWheel,            ///< Wheel mode (wheel-leg)
};

/**
 * @class autodriver::chassis::OperationalModeController
 * @brief Thread-safe mode FSM used by ChassisManager / SafetyGate.
 */
class OperationalModeController {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(OperationalModeController)

  /**
   * @brief Disable copy construction and copy assignment.
   */
  DISALLOW_COPY_AND_ASSIGN(OperationalModeController)

  /**
   * @brief Construct in Idle with unspecified locomotion intent.
   */
  OperationalModeController() = default;

  /**
   * @brief Current operational mode.
   * @return Snapshot of the FSM state.
   */
  OperationalMode mode() const;

  /**
   * @brief Current locomotion intent (stand/walk/wheel).
   * @return Last accepted intent, or kUnspecified.
   */
  LocomotionIntent intent() const;

  /**
   * @brief Stable string for RobotState.active_cmd_id / mode_state channel.
   * @return Lowercase mode name (e.g. "armed").
   */
  std::string ModeString() const;

  /**
   * @brief Stable string for locomotion intent.
   * @return Lowercase intent name (e.g. "walk").
   */
  std::string IntentString() const;

  /**
   * @brief Handle a mode command from Autolink (std_msgs/String.data).
   *
   * Accepted: arm, disarm, estop, clear_estop, clear_fault, dock, undock,
   * stand, walk, wheel.
   * @param[in] command Raw command text (trimmed, case-insensitive).
   * @param[out] error Optional human-readable failure reason.
   * @return true when the transition / intent update was accepted.
   */
  bool HandleModeCommand(const std::string& command,
                         std::string* error = nullptr);

  /**
   * @brief Notify that a twist was applied (Armed→Moving when non-zero).
   * @param[in] non_zero_twist true when the command has meaningful velocity.
   */
  void NotifyMotionApplied(bool non_zero_twist);

  /**
   * @brief Notify vendor fault (→ Fault). Does not clear EStop.
   */
  void NotifyFault();

  /**
   * @brief Notify hardware E-stop (→ EStop).
   */
  void NotifyEStop();

  /**
   * @brief True when SafetyGate may forward twist to the driver.
   * @param[in] require_arm When false, Idle is treated like Armed for motion.
   * @return false in Fault / EStop (and Idle when require_arm).
   */
  bool AllowsMotion(bool require_arm) const;

private:
  mutable std::mutex mutex_;  ///< Guards mode_ / intent_.
  OperationalMode mode_{OperationalMode::kIdle};
  LocomotionIntent intent_{LocomotionIntent::kUnspecified};
};

/**
 * @brief Parse OperationalMode from string (for tests / logs).
 * @param[in] text Mode name (case-insensitive).
 * @return Parsed mode, or kIdle when unrecognized.
 */
OperationalMode ParseOperationalMode(const std::string& text);

/**
 * @brief Lowercase name of @p mode.
 * @param[in] mode Operational mode enum.
 * @return Stable C string (never null).
 */
const char* OperationalModeToString(OperationalMode mode);

/**
 * @brief Lowercase name of @p intent.
 * @param[in] intent Locomotion intent enum.
 * @return Stable C string (never null).
 */
const char* LocomotionIntentToString(LocomotionIntent intent);

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_OPERATIONAL_MODE_HPP_

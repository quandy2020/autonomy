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
 * @file chassis_driver.hpp
 * @brief Abstract robot-body hardware backend (vendor SDK boundary).
 *
 * Wire bodies: TwistStamped / RobotState / RobotEvent (automsgs). Optional
 * locomotion intent + tool commands are Manager-routed hooks with defaults.
 */

#ifndef AUTODRIVER_CHASSIS_CHASSIS_DRIVER_HPP_
#define AUTODRIVER_CHASSIS_CHASSIS_DRIVER_HPP_

#include <functional>

#include "chassis/operational_mode.hpp"
#include "chassis/tool_command.hpp"
#include "chassis/types.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace chassis {

/**
 * @class autodriver::chassis::ChassisDriver
 * @brief Vendor plugin: twist in, RobotState out (+ optional events / tools).
 *
 * Created by ChassisBackendRegistry (owning raw pointer → SharedPtr).
 * ChassisManager owns the driver and routes SafetyGate / mode / tool traffic.
 */
class ChassisDriver {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(ChassisDriver)

  /**
   * @brief Disable copy construction and copy assignment.
   */
  DISALLOW_COPY_AND_ASSIGN(ChassisDriver)

  /**
   * @brief Callback type for asynchronous vendor RobotEvent reports
   *        (FAULT / E-STOP / BATTERY_LOW, etc.).
   * @param[in] event Event body to forward (e.g. to ChassisManager / Autolink).
   */
  using EventCallback = std::function<void(const ChassisEvent&)>;

  /**
   * @brief Virtual destructor for polymorphic vendor drivers.
   */
  virtual ~ChassisDriver() = default;

  /**
   * @brief Stable chassis instance id from configuration / factory.
   * @return Identifier such as "chassis/base".
   */
  virtual const ChassisId& GetChassisId() const = 0;

  /**
   * @brief Open the device / SDK session and enable motion when hardware allows.
   * @return true on success; false if the device cannot be opened.
   */
  virtual bool Start() = 0;

  /**
   * @brief Soft-stop actuators and release SDK / device resources.
   */
  virtual void Stop() = 0;

  /**
   * @brief Query whether the SDK session is open after Start().
   * @return true while the motion session is active.
   */
  virtual bool IsRunning() const = 0;

  /**
   * @brief Apply a body-frame velocity command (geometry_msgs TwistStamped).
   *
   * Zero twist means soft stop. ChassisManager watchdog also applies zero twist
   * when cmd_vel times out.
   * @param[in] command Desired body twist (linear / angular); header stamp optional.
   * @return false if not running or motion is disabled (e.g. after E-stop).
   */
  virtual bool ApplyVelocityCommand(const ChassisCommand& command) = 0;

  /**
   * @brief Optional locomotion intent for legged / wheel-leg / humanoid backends.
   * @param[in] intent stand / walk / wheel (or unspecified).
   * @return true when accepted or ignored harmlessly (default implementation).
   */
  virtual bool ApplyLocomotionIntent(LocomotionIntent /*intent*/) {
    return true;
  }

  /**
   * @brief Optional job tool command (brush / blade / door) — never locomotion.
   * @param[in] command Parsed tool name / enable / value.
   * @return false when unsupported (default implementation).
   */
  virtual bool ApplyToolCommand(const ToolCommand& /*command*/) {
    return false;
  }

  /**
   * @brief Read the latest chassis state into @p state (vehicle_msgs.RobotState).
   *
   * Fills pose / twist / battery / motion flags. Task-level fields may stay
   * default — autonomy fills those at the bridge layer. Manager may overwrite
   * @c active_cmd_id with the operational mode name.
   * @param[out] state Destination RobotState; must be non-null.
   * @return false if @p state is null or the driver cannot sample.
   */
  virtual bool ReadChassisState(ChassisState* state) = 0;

  /**
   * @brief Trigger hardware emergency stop: halt actuators and set
   *        motion_enabled=false (may also EmitChassisEvent).
   * @return true if the E-stop request was accepted by the vendor SDK.
   */
  virtual bool TriggerEmergencyStop() = 0;

  /**
   * @brief Register a callback for asynchronous vendor RobotEvent pushes.
   *
   * Optional; default implementation stores @p callback for EmitChassisEvent().
   * @param[in] callback Invoked on the driver thread; may be empty to clear.
   */
  virtual void SetEventCallback(EventCallback callback) {
    event_callback_ = std::move(callback);
  }

protected:
  /**
   * @brief Protected default constructor for derived vendor drivers.
   */
  ChassisDriver() = default;

  /**
   * @brief Forward a RobotEvent to the registered EventCallback.
   * @param[in] event Event to deliver; no-op when no callback is set.
   */
  void EmitChassisEvent(const ChassisEvent& event) {
    if (event_callback_) {
      event_callback_(event);
    }
  }

  /** @brief Optional async event sink (ChassisManager / tests). */
  EventCallback event_callback_;
};

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_CHASSIS_DRIVER_HPP_

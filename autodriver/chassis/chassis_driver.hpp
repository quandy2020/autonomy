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
 * @brief Abstract robot-body hardware backend (vendor SDK boundary).
 *
 * Message bodies are automsgs vehicle_msgs (RobotState / RobotEvent) and
 * TwistStamped — identical to the rest of the stack. No autonomy/vehicle.
 */

#ifndef AUTODRIVER_CHASSIS_CHASSIS_DRIVER_HPP_
#define AUTODRIVER_CHASSIS_CHASSIS_DRIVER_HPP_

#include <functional>
#include <vector>

#include "chassis/types.hpp"

namespace autodriver {
namespace chassis {

/**
 * @class ChassisDriver
 * @brief Vendor plugin: TwistStamped in, RobotState out (+ optional events).
 */
class ChassisDriver {
public:
  using EventCallback = std::function<void(const ChassisEvent&)>;

  ChassisDriver(const ChassisDriver&) = delete;
  ChassisDriver& operator=(const ChassisDriver&) = delete;
  virtual ~ChassisDriver() = default;

  virtual const ChassisId& GetChassisId() const = 0;

  /** Open device / SDK session. */
  virtual bool Start() = 0;
  /** Stop motion and release resources. */
  virtual void Stop() = 0;
  virtual bool IsRunning() const = 0;

  /**
   * @brief Apply velocity command (vehicle_msgs-compatible TwistStamped).
   * Zero twist = soft stop. Watchdog also sends zero twist.
   */
  virtual bool ApplyCommand(const ChassisCommand& command) = 0;

  /**
   * @brief Fill vehicle_msgs.RobotState (pose / twist / battery / flags).
   * Task fields may be left default — autonomy fills those at bridge layer.
   */
  virtual bool GetState(ChassisState* state) = 0;

  /** Hardware E-stop: stop actuators and set motion_enabled=false. */
  virtual bool EmergencyStop() = 0;

  /** Optional: vendor may push RobotEvent (FAULT / E-STOP / BATTERY_LOW). */
  virtual void SetEventCallback(EventCallback callback) {
    event_callback_ = std::move(callback);
  }

protected:
  ChassisDriver() = default;

  void EmitEvent(const ChassisEvent& event) {
    if (event_callback_) {
      event_callback_(event);
    }
  }

  EventCallback event_callback_;
};

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_CHASSIS_DRIVER_HPP_

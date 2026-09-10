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
 * Decoupled from autonomy/vehicle: that module stays a kinematics / model
 * abstraction inside the autonomy process. ChassisDriver owns real hardware.
 */

#ifndef AUTODRIVER_CHASSIS_CHASSIS_DRIVER_HPP_
#define AUTODRIVER_CHASSIS_CHASSIS_DRIVER_HPP_

#include "autodriver/chassis/types.hpp"

namespace autodriver {
namespace chassis {

/**
 * @class ChassisDriver
 * @brief Vendor plugin: command in, state out. No Autolink, no autonomy/*.
 */
class ChassisDriver {
public:
  ChassisDriver(const ChassisDriver&) = delete;
  ChassisDriver& operator=(const ChassisDriver&) = delete;
  virtual ~ChassisDriver() = default;

  virtual const ChassisId& GetChassisId() const = 0;
  virtual ChassisKind GetKind() const = 0;

  /** Open device / SDK session. */
  virtual bool Start() = 0;
  /** Stop motion and release resources. */
  virtual void Stop() = 0;
  virtual bool IsRunning() const = 0;

  /**
   * @brief Apply a motion command (may be zero after watchdog).
   * @return false on hardware reject / fault.
   */
  virtual bool ApplyCommand(const ChassisCommand& command) = 0;

  /**
   * @brief Read latest chassis state (odom, battery, faults).
   * @return false if state unavailable.
   */
  virtual bool GetState(ChassisState* state) = 0;

  /** Immediate safe stop (E-stop / fault path). */
  virtual bool EmergencyStop() {
    ChassisCommand stop;
    stop.emergency_stop = true;
    return ApplyCommand(stop);
  }

protected:
  ChassisDriver() = default;
};

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_CHASSIS_DRIVER_HPP_

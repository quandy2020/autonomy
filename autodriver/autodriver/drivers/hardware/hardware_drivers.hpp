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
 * @brief Factory for serial/CAN hardware sensor drivers.
 */

#ifndef AUTODRIVER_DRIVERS_HARDWARE_HARDWARE_DRIVERS_HPP_
#define AUTODRIVER_DRIVERS_HARDWARE_HARDWARE_DRIVERS_HPP_

#include <memory>
#include <string>

#include "autodriver/common/sensor_id.hpp"
#include "autodriver/config/hub_config.hpp"
#include "autodriver/drivers/hardware/driver_params.hpp"
#include "autodriver/hal/sensor_driver.hpp"

namespace autodriver {
namespace hardware {

/** @brief Registers serial_*, can_*, and realsense_* drivers with SensorFactory. */
void RegisterHardwareDrivers();

/**
 * @brief Creates a hardware driver from factory name and params.
 * @return nullptr if factory_name is unknown.
 */
std::shared_ptr<SensorDriver> CreateHardwareDriver(
  const std::string & factory_name,
  const SensorId & sensor_id,
  const DriverParams & params);

/** @brief Creates a driver from a full DriverConfig entry. */
std::shared_ptr<SensorDriver> CreateFromConfig(const DriverConfig & config);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_DRIVERS_HARDWARE_HARDWARE_DRIVERS_HPP_

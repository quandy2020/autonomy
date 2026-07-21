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
 * @brief Unified driver creation (mock + hardware).
 */

#ifndef AUTODRIVER_DRIVERS_DRIVER_REGISTRY_HPP_
#define AUTODRIVER_DRIVERS_DRIVER_REGISTRY_HPP_

#include <memory>

#include "autodriver/config/hub_config.hpp"
#include "autodriver/hal/sensor_driver.hpp"

namespace autodriver {
namespace drivers {

/** @brief Registers hardware factories and optionally mock factories. */
void RegisterDriverFactories(bool register_mocks);

/** @brief Registers mock and hardware driver factories. */
void RegisterBuiltinDrivers();

/** @brief Instantiates a driver from HubConfig entry (hardware first, then mock). */
std::shared_ptr<SensorDriver> CreateDriver(
  const DriverConfig & config,
  bool allow_mock = true);

}  // namespace drivers
}  // namespace autodriver

#endif  // AUTODRIVER_DRIVERS_DRIVER_REGISTRY_HPP_

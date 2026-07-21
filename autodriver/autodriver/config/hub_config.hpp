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
 * @brief Declarative SensorHub and driver registration settings.
 */

#ifndef AUTODRIVER_CONFIG_HUB_CONFIG_HPP_
#define AUTODRIVER_CONFIG_HUB_CONFIG_HPP_

#include <string>
#include <vector>

#include "autodriver/common/sensor_id.hpp"
#include "autodriver/drivers/hardware/driver_params.hpp"
#include "autodriver/sync/sensor_hub.hpp"

namespace autodriver {

/**
 * @struct DriverConfig
 * @brief One driver instance created through SensorFactory.
 */
struct DriverConfig
{
  /** Registered factory name, e.g. "mock_imu". */
  std::string factory_name;
  /** Instance id stamped on outgoing samples. */
  SensorId sensor_id;
  /** Driver-specific options (device, baud, can_id, scales, ...). */
  hardware::DriverParams params;
};

/**
 * @struct HubConfig
 * @brief Full HAL stack configuration for SensorManager.
 */
struct HubConfig
{
  SensorHubOptions hub_options;
  std::vector<DriverConfig> drivers;
  /** When true, registers built-in mock_* drivers before loading entries. */
  bool register_builtin_mocks{false};
};

/**
 * @brief Default mock stack for desktop simulation (IMU + wheel + LiDAR).
 */
HubConfig DefaultSimulationHubConfig();

}  // namespace autodriver

#endif  // AUTODRIVER_CONFIG_HUB_CONFIG_HPP_

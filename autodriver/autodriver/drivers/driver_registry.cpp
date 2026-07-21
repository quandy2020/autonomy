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
 * @brief Implements driver registry.
 */

#include "autodriver/drivers/driver_registry.hpp"

#include "autodriver/drivers/hardware/hardware_drivers.hpp"
#include "autodriver/drivers/mock/mock_drivers.hpp"

namespace autodriver {
namespace drivers {

void RegisterDriverFactories(bool register_mocks)
{
  hardware::RegisterHardwareDrivers();
  if (register_mocks) {
    mock::RegisterMockDrivers();
  }
}

void RegisterBuiltinDrivers()
{
  RegisterDriverFactories(true);
}

std::shared_ptr<SensorDriver> CreateDriver(
  const DriverConfig & config,
  const bool allow_mock)
{
  if (auto driver = hardware::CreateFromConfig(config)) {
    return driver;
  }
  if (!allow_mock) {
    return nullptr;
  }
  return mock::CreateByName(config.factory_name, config.sensor_id);
}

}  // namespace drivers
}  // namespace autodriver

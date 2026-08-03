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
 * @brief High-level lifecycle API for the sensor HAL stack.
 */

#ifndef AUTODRIVER_APP_SENSOR_MANAGER_HPP_
#define AUTODRIVER_APP_SENSOR_MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "autodriver/config/hub_config.hpp"
#include "autodriver/sync/sensor_hub.hpp"

namespace autodriver {

/**
 * @class SensorManager
 * @brief Loads HubConfig, instantiates drivers, and owns SensorHub.
 *
 * Typical flow: Initialize()-> Set*Callback()-> Start()-> Stop().
 */
class SensorManager
{
public:
  SensorManager();
  explicit SensorManager(HubConfig config);

  /** @brief Applies config: registers mock factories and driver instances. */
  bool Initialize();

  /** @brief Starts SensorHub and all registered drivers. */
  bool Start();

  /** @brief Stops SensorHub and drivers. */
  void Stop();

  /** @brief Returns true after a successful Start() until Stop(). */
  bool IsRunning() const;

  /** @brief Mutable hub for advanced callback wiring before Start(). */
  SensorHub & hub();

  /** @brief Read-only hub access. */
  const SensorHub & hub() const;

  /** @brief Driver factory names that failed to load during Initialize(). */
  const std::vector<std::string> & init_errors() const;

  void SetAlignedCallback(SensorHub::AlignedCallback callback);
  void SetRawSampleCallback(SensorHub::RawSampleCallback callback);

private:
  bool CreateDriver(const DriverConfig & entry);

  HubConfig config_;
  SensorHub hub_;
  std::vector<std::string> init_errors_;
  bool initialized_{false};
};

}  // namespace autodriver

#endif  // AUTODRIVER_APP_SENSOR_MANAGER_HPP_

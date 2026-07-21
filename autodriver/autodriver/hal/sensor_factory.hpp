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
 * @brief Factory registry for plug-in sensor drivers.
 */

#ifndef AUTODRIVER_HAL_SENSOR_FACTORY_HPP_
#define AUTODRIVER_HAL_SENSOR_FACTORY_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "autodriver/hal/sensor_driver.hpp"

namespace autodriver {

/**
 * @class SensorFactory
 * @brief Thread-safe registry mapping driver names to constructors.
 *
 * Backends register at static init or startup, e.g.
 * Register("mock_imu", [] { return std::make_shared<MockImuDriver>(...); });
 */
class SensorFactory
{
public:
  using Creator = std::function<std::shared_ptr<SensorDriver>()>;

  /** @brief Returns the process-wide factory singleton. */
  static SensorFactory & Instance();

  /**
   * @brief Registers a driver constructor under a unique name.
   * @return false if name already registered.
   */
  bool Register(const std::string & name, Creator creator);

  /**
   * @brief Instantiates a driver by registered name.
   * @return nullptr if name is unknown.
   */
  std::shared_ptr<SensorDriver> Create(const std::string & name) const;

  /** @brief Returns true when a name has been registered. */
  bool Has(const std::string & name) const;

private:
  SensorFactory() = default;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, Creator> creators_;
};

}  // namespace autodriver

#endif  // AUTODRIVER_HAL_SENSOR_FACTORY_HPP_

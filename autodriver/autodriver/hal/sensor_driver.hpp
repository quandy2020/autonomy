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
 * @brief Abstract sensor driver interface for HAL backends.
 */

#ifndef AUTODRIVER_HAL_SENSOR_DRIVER_HPP_
#define AUTODRIVER_HAL_SENSOR_DRIVER_HPP_

#include <functional>
#include <memory>
#include <string>

#include "autodriver/common/sensor_id.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autodriver/types/sensor_type.hpp"

namespace autodriver {

/**
 * @class SensorDriver
 * @brief Plug-in interface implemented by each sensor backend.
 *
 * Drivers push timestamped samples through a callback registered via
 * SetSampleCallback(). SensorHub owns time synchronization and alignment.
 */
class SensorDriver
{
public:
  using SampleCallback =
    std::function<void(std::unique_ptr<SensorSample> sample)>;

  virtual ~SensorDriver() = default;

protected:
  SensorDriver() = default;

public:
  SensorDriver(const SensorDriver &) = delete;
  SensorDriver & operator=(const SensorDriver &) = delete;

  /** @brief Sensor modality provided by this driver. */
  virtual SensorType GetType() const = 0;

  /** @brief Unique instance id, e.g. "lidar/front". */
  virtual const SensorId & GetSensorId() const = 0;

  /** @brief Starts acquisition; returns false on failure. */
  virtual bool Start() = 0;

  /** @brief Stops acquisition and releases hardware resources. */
  virtual void Stop() = 0;

  /** @brief Returns true while the driver is actively publishing. */
  virtual bool IsRunning() const = 0;

  /**
   * @brief Registers the callback invoked for each new sample.
   * @param callback Called on the driver's thread; must be thread-safe if shared.
   */
  virtual void SetSampleCallback(SampleCallback callback) = 0;
};

}  // namespace autodriver

#endif  // AUTODRIVER_HAL_SENSOR_DRIVER_HPP_

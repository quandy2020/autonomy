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
 * @brief Base class for timestamped sensor samples.
 */

#ifndef AUTODRIVER_TYPES_SENSOR_SAMPLE_HPP_
#define AUTODRIVER_TYPES_SENSOR_SAMPLE_HPP_

#include <memory>
#include <utility>

#include "autodriver/common/sensor_id.hpp"
#include "autodriver/common/time.hpp"
#include "autodriver/types/sensor_type.hpp"

namespace autodriver {

/**
 * @class SensorSample
 * @brief Polymorphic base for all HAL sensor payloads.
 *
 * Each sample carries device time (from the sensor clock) and host time
 * (after TimeSync correction). Upper layers should prefer host_time().
 */
class SensorSample
{
public:
  SensorSample(SensorId sensor_id, SensorType type, Timestamp device_time)
  : sensor_id_(std::move(sensor_id)),
    type_(type),
    device_time_(device_time),
    host_time_(device_time)
  {
  }

  virtual ~SensorSample() = default;

  /**
   * @brief Creates a deep copy of this sample.
   */
  virtual std::unique_ptr<SensorSample> Clone() const = 0;

  /** @brief Sensor modality. */
  SensorType type() const { return type_; }

  /** @brief Unique sensor instance id. */
  const SensorId & sensor_id() const { return sensor_id_; }

  /** @brief Timestamp reported by the device. */
  Timestamp device_time() const { return device_time_; }

  /** @brief Host-aligned timestamp after TimeSync. */
  Timestamp host_time() const { return host_time_; }

  /** @brief Sets the host-aligned timestamp (called by SensorHub). */
  void set_host_time(Timestamp host_time) { host_time_ = host_time; }

protected:
  SensorId sensor_id_;
  SensorType type_;
  Timestamp device_time_;
  Timestamp host_time_;
};

}  // namespace autodriver

#endif  // AUTODRIVER_TYPES_SENSOR_SAMPLE_HPP_

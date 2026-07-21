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
 * @brief Per-sensor device-to-host time offset estimation.
 */

#ifndef AUTODRIVER_SYNC_TIME_SYNC_HPP_
#define AUTODRIVER_SYNC_TIME_SYNC_HPP_

#include <cstdint>
#include <mutex>
#include <unordered_map>

#include "autodriver/common/sensor_id.hpp"
#include "autodriver/common/time.hpp"

namespace autodriver {

/**
 * @class TimeSync
 * @brief Maintains device-to-host clock offsets per sensor.
 *
 * Offsets are updated from paired (device_time, host_receive_time) observations
 * using exponential smoothing. host_time = device_time + offset.
 */
class TimeSync
{
public:
  /**
   * @brief Updates the offset estimate for a sensor.
   * @param sensor_id Sensor instance id.
   * @param device_time Timestamp from the device payload.
   * @param host_receive_time Wall time when the sample arrived on the host.
   */
  void Update(
    const SensorId & sensor_id,
    Timestamp device_time,
    Timestamp host_receive_time);

  /**
   * @brief Maps a device timestamp to host time.
   * @return host_receive_time if no offset has been learned yet.
   */
  Timestamp ToHostTime(
    const SensorId & sensor_id,
    Timestamp device_time) const;

  /** @brief Clears all learned offsets. */
  void Reset();

  /** @brief Returns the current offset in nanoseconds (0 if unknown). */
  int64_t GetOffsetNanoseconds(const SensorId & sensor_id) const;

private:
  struct OffsetState {
    int64_t offset_ns{0};
    bool initialized{false};
  };

  mutable std::mutex mutex_;
  std::unordered_map<SensorId, OffsetState> offsets_;
  static constexpr double kSmoothingAlpha = 0.2;
};

}  // namespace autodriver

#endif  // AUTODRIVER_SYNC_TIME_SYNC_HPP_

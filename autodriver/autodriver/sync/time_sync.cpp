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
 * @brief Implements TimeSync.
 */

#include "autodriver/sync/time_sync.hpp"

#include <cmath>

namespace autodriver {

void TimeSync::Update(
  const SensorId & sensor_id,
  Timestamp device_time,
  Timestamp host_receive_time)
{
  const int64_t observed_offset =
    ToNanoseconds(host_receive_time) - ToNanoseconds(device_time);

  std::lock_guard<std::mutex> lock(mutex_);
  OffsetState & state = offsets_[sensor_id];
  if (!state.initialized) {
    state.offset_ns = observed_offset;
    state.initialized = true;
    return;
  }

  state.offset_ns = static_cast<int64_t>(std::llround(
      (1.0 - kSmoothingAlpha) * static_cast<double>(state.offset_ns) +
      kSmoothingAlpha * static_cast<double>(observed_offset)));
}

Timestamp TimeSync::ToHostTime(
  const SensorId & sensor_id,
  Timestamp device_time) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = offsets_.find(sensor_id);
  if (it == offsets_.end() || !it->second.initialized) {
    return device_time;
  }
  return FromNanoseconds(
    ToNanoseconds(device_time) + it->second.offset_ns);
}

void TimeSync::Reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  offsets_.clear();
}

int64_t TimeSync::GetOffsetNanoseconds(const SensorId & sensor_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = offsets_.find(sensor_id);
  if (it == offsets_.end()) {
    return 0;
  }
  return it->second.offset_ns;
}

}  // namespace autodriver

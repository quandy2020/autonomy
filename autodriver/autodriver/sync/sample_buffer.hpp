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
 * @brief Time-ordered per-sensor sample ring buffer.
 */

#ifndef AUTODRIVER_SYNC_SAMPLE_BUFFER_HPP_
#define AUTODRIVER_SYNC_SAMPLE_BUFFER_HPP_

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>

#include "autodriver/common/sensor_id.hpp"
#include "autodriver/common/time.hpp"
#include "autodriver/types/sensor_sample.hpp"

namespace autodriver {

/**
 * @class SampleBuffer
 * @brief Stores the latest samples for one sensor, sorted by host time.
 */
class SampleBuffer
{
public:
  explicit SampleBuffer(std::size_t capacity = 32);

  /** @brief Inserts a sample; drops oldest entries when over capacity. */
  void Push(std::unique_ptr<SensorSample> sample);

  /**
   * @brief Returns the newest sample with host_time <= reference_time.
   * @param reference_time Alignment anchor on the host clock.
   */
  std::unique_ptr<SensorSample> LatestAtOrBefore(Timestamp reference_time) const;

  /** @brief Returns the most recent sample regardless of time. */
  std::unique_ptr<SensorSample> Latest() const;

  /** @brief Removes all buffered samples. */
  void Clear();

  /** @brief Number of samples currently buffered. */
  std::size_t Size() const;

private:
  mutable std::mutex mutex_;
  std::size_t capacity_;
  std::deque<std::unique_ptr<SensorSample>> samples_;
};

}  // namespace autodriver

#endif  // AUTODRIVER_SYNC_SAMPLE_BUFFER_HPP_

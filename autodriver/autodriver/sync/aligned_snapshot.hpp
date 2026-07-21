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
 * @brief Time-aligned multi-sensor snapshot for upper-layer consumers.
 */

#ifndef AUTODRIVER_SYNC_ALIGNED_SNAPSHOT_HPP_
#define AUTODRIVER_SYNC_ALIGNED_SNAPSHOT_HPP_

#include <memory>
#include <unordered_map>

#include "autodriver/common/time.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autodriver/types/sensor_type.hpp"

namespace autodriver {

/**
 * @struct AlignedSnapshot
 * @brief One host-time anchor with the latest sample per sensor type.
 *
 * Samples are deep-copied clones suitable for passing across threads.
 */
struct AlignedSnapshot
{
  /** Host clock anchor used for alignment. */
  Timestamp reference_time;

  /** Latest sample per modality at or before reference_time. */
  std::unordered_map<SensorType, std::unique_ptr<SensorSample>> samples;

  /** @brief Returns a const pointer to a typed sample, or nullptr. */
  template<typename SampleT>
  const SampleT * Get(SensorType type) const
  {
    const auto it = samples.find(type);
    if (it == samples.end() || !it->second) {
      return nullptr;
    }
    return dynamic_cast<const SampleT *>(it->second.get());
  }
};

}  // namespace autodriver

#endif  // AUTODRIVER_SYNC_ALIGNED_SNAPSHOT_HPP_

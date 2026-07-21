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
 * @brief Standardized single-beam range sample for HAL consumers.
 */

#ifndef AUTODRIVER_TYPES_RANGE_SAMPLE_HPP_
#define AUTODRIVER_TYPES_RANGE_SAMPLE_HPP_

#include <memory>

#include "autodriver/types/sensor_sample.hpp"

namespace autodriver {

/**
 * @struct RangeSample
 * @brief Distance from an ultrasonic/IR/time-of-flight sensor [m].
 */
struct RangeSample : public SensorSample
{
  double range_m{0.0};
  /** Minimum valid range [m]. */
  double range_min{0.0};
  /** Maximum valid range [m]. */
  double range_max{0.0};

  RangeSample(
    SensorId sensor_id,
    Timestamp device_time,
    double range_m,
    double range_min,
    double range_max)
  : SensorSample(std::move(sensor_id), SensorType::kRangeFinder, device_time),
    range_m(range_m),
    range_min(range_min),
    range_max(range_max)
  {
  }

  std::unique_ptr<SensorSample> Clone() const override
  {
    return std::make_unique<RangeSample>(*this);
  }
};

}  // namespace autodriver

#endif  // AUTODRIVER_TYPES_RANGE_SAMPLE_HPP_

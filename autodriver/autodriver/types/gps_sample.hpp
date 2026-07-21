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
 * @brief Standardized GNSS sample for HAL consumers.
 */

#ifndef AUTODRIVER_TYPES_GPS_SAMPLE_HPP_
#define AUTODRIVER_TYPES_GPS_SAMPLE_HPP_

#include <cstdint>
#include <memory>

#include "autodriver/types/sensor_sample.hpp"

namespace autodriver {

/**
 * @enum GpsFixStatus
 * @brief GNSS fix quality indicator.
 */
enum class GpsFixStatus : std::uint8_t
{
  kNoFix = 0,
  kFix2D,
  kFix3D,
  kRtkFloat,
  kRtkFixed,
};

/**
 * @struct GpsSample
 * @brief WGS84 latitude/longitude/altitude with fix status.
 */
struct GpsSample : public SensorSample
{
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_m{0.0};
  GpsFixStatus fix_status{GpsFixStatus::kNoFix};

  GpsSample(
    SensorId sensor_id,
    Timestamp device_time,
    double latitude_deg,
    double longitude_deg,
    double altitude_m,
    GpsFixStatus fix_status)
  : SensorSample(std::move(sensor_id), SensorType::kGps, device_time),
    latitude_deg(latitude_deg),
    longitude_deg(longitude_deg),
    altitude_m(altitude_m),
    fix_status(fix_status)
  {
  }

  std::unique_ptr<SensorSample> Clone() const override
  {
    return std::make_unique<GpsSample>(*this);
  }
};

}  // namespace autodriver

#endif  // AUTODRIVER_TYPES_GPS_SAMPLE_HPP_

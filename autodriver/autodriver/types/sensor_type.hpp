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
 * @brief Sensor modality enumeration for HAL registration and alignment.
 */

#ifndef AUTODRIVER_TYPES_SENSOR_TYPE_HPP_
#define AUTODRIVER_TYPES_SENSOR_TYPE_HPP_

#include <cstdint>

namespace autodriver {

/**
 * @enum SensorType
 * @brief Supported sensor modalities in autodriver HAL.
 */
enum class SensorType : std::uint8_t
{
  kImu = 0,
  kGps,
  kWheelOdometry,
  kCamera,
  kLidar,
  kRangeFinder,
};

}  // namespace autodriver

#endif  // AUTODRIVER_TYPES_SENSOR_TYPE_HPP_

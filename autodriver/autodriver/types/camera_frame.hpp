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
 * @brief Standardized camera frame for HAL consumers.
 */

#ifndef AUTODRIVER_TYPES_CAMERA_FRAME_HPP_
#define AUTODRIVER_TYPES_CAMERA_FRAME_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "autodriver/types/sensor_sample.hpp"

namespace autodriver {

/**
 * @struct CameraFrame
 * @brief Raw image buffer with encoding metadata.
 */
struct CameraFrame : public SensorSample
{
  std::uint32_t width{0};
  std::uint32_t height{0};
  /** Pixel encoding, e.g. "rgb8", "bgr8", "mono8". */
  std::string encoding;
  std::vector<std::uint8_t> data;

  CameraFrame(
    SensorId sensor_id,
    Timestamp device_time,
    std::uint32_t width,
    std::uint32_t height,
    std::string encoding,
    std::vector<std::uint8_t> data)
  : SensorSample(std::move(sensor_id), SensorType::kCamera, device_time),
    width(width),
    height(height),
    encoding(std::move(encoding)),
    data(std::move(data))
  {
  }

  std::unique_ptr<SensorSample> Clone() const override
  {
    return std::make_unique<CameraFrame>(*this);
  }
};

}  // namespace autodriver

#endif  // AUTODRIVER_TYPES_CAMERA_FRAME_HPP_

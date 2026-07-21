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
 * @brief Standardized wheel odometry sample for HAL consumers.
 */

#ifndef AUTODRIVER_TYPES_WHEEL_ODOMETRY_SAMPLE_HPP_
#define AUTODRIVER_TYPES_WHEEL_ODOMETRY_SAMPLE_HPP_

#include <memory>

#include "autodriver/types/sensor_sample.hpp"

namespace autodriver {

/**
 * @struct WheelOdometrySample
 * @brief Planar velocity and yaw rate from wheel encoders.
 */
struct WheelOdometrySample : public SensorSample
{
  /** Forward speed [m/s] in the robot base frame. */
  double linear_x{0.0};
  /** Lateral speed [m/s]; zero for differential-drive platforms. */
  double linear_y{0.0};
  /** Yaw rate [rad/s]. */
  double angular_z{0.0};

  WheelOdometrySample(
    SensorId sensor_id,
    Timestamp device_time,
    double linear_x,
    double linear_y,
    double angular_z)
  : SensorSample(std::move(sensor_id), SensorType::kWheelOdometry, device_time),
    linear_x(linear_x),
    linear_y(linear_y),
    angular_z(angular_z)
  {
  }

  std::unique_ptr<SensorSample> Clone() const override
  {
    return std::make_unique<WheelOdometrySample>(*this);
  }
};

}  // namespace autodriver

#endif  // AUTODRIVER_TYPES_WHEEL_ODOMETRY_SAMPLE_HPP_

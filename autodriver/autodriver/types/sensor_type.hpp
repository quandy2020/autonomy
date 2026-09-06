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
 * @brief Sensor modality enumeration.
 */

#ifndef AUTODRIVER_TYPES_SENSOR_TYPE_HPP_
#define AUTODRIVER_TYPES_SENSOR_TYPE_HPP_

#include <cstdint>

namespace autodriver {

/**
 * @brief Supported sensor modalities in autodriver.
 */
enum class SensorType : std::uint8_t {
    /**
     * @brief Inertial measurement unit.
     */
    kImu = 0,
    /**
     * @brief Global navigation satellite receiver.
     */
    kGps,
    /**
     * @brief Camera image stream.
     */
    kCamera,
    /**
     * @brief Two-dimensional lidar scan.
     */
    kLidar2d,
    /**
     * @brief Three-dimensional point cloud.
     */
    kLidar3d,
    /**
     * @brief Single-beam or narrow range sensor.
     */
    kRangeFinder,
    /**
     * @brief Wheel encoder odometry.
     */
    kWheelOdometry,
    /**
     * @brief Automotive radar (object list / clusters).
     */
    kRadar,
    /**
     * @brief Multi-channel microphone / audio chunk.
     */
    kMicrophone,
};

}  // namespace autodriver

#endif  // AUTODRIVER_TYPES_SENSOR_TYPE_HPP_

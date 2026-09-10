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
 * @brief Serial WIT-motion IMU driver.
 */

#ifndef AUTODRIVER_IMU_SERIAL_IMU_DRIVER_HPP_
#define AUTODRIVER_IMU_SERIAL_IMU_DRIVER_HPP_

#include <cstddef>
#include <cstdint>

#include "autodriver/common/serial_byte_driver_base.hpp"
#include "autodriver/driver_params.hpp"
#include "autodriver/imu/wit_motion_parser.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace hardware {

/**
 * @class autodriver::hardware::SerialImuDriver
 * @brief WIT-motion 0x55 via SerialByteDriverBase + WitMotionParser.
 *
 * Params: `device`, `baud`, `accel_scale`, `gyro_scale`.
 */
class SerialImuDriver : public SerialByteDriverBase<SerialImuDriver> {
public:
    /**
     * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
     */
    AUTOLINK_SHARED_PTR_DEFINITIONS(SerialImuDriver)

    /**
     * @brief Store identity and construct WitMotionParser scale factors.
     * @param id Sensor instance id.
     * @param params DriverParams (cold-path parse only).
     */
    SerialImuDriver(SensorId id, DriverParams params);

    /**
     * @brief Stops the reader thread before destroying the parser.
     */
    ~SerialImuDriver() override { Stop(); }

    /**
     * @brief Report sensor type.
     * @return SensorType::kImu.
     */
    SensorType GetSensorType() const override { return SensorType::kImu; }

    /**
     * @brief CRTP hook: feed WIT parser and emit ImuSample when complete.
     * @param data Bytes read from the serial Stream.
     * @param n Number of valid bytes in @p data.
     */
    void OnBytes(const std::uint8_t* data, std::size_t n);

private:
    // Incremental WIT-motion protocol parser.
    protocol::WitMotionParser parser_;
};

/**
 * @brief Factory for ImuBackendRegistry (REGISTER_IMU_BACKEND "serial").
 * @param id Sensor instance id from YAML.
 * @param params Backend-specific key/value map.
 * @return Owning SerialImuDriver* (never null).
 */
SensorDriver* CreateSerialImuDriver(const SensorId& id,
                                    const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_IMU_SERIAL_IMU_DRIVER_HPP_

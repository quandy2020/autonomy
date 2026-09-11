/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file can_imu_driver.hpp
 * @brief SocketCAN IMU driver (scaled int16 triplet frames).
 */

#ifndef AUTODRIVER_IMU_CAN_IMU_DRIVER_HPP_
#define AUTODRIVER_IMU_CAN_IMU_DRIVER_HPP_

#include <array>
#include <cstdint>

#include "autodriver/common/can_sensor_driver_base.hpp"
#include "autodriver/driver_params.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace hardware {

/**
 * @struct autodriver::hardware::ImuCanEvent
 * @brief Partial accel/gyro event from one CAN frame (fused in driver).
 */
struct ImuCanEvent {
    /**
     * @brief Which half of the IMU this frame carries.
     */
    enum class Kind : std::uint8_t { kAccel, kGyro };

    /** @brief Accel or gyro payload. */
    Kind kind = Kind::kAccel;

    /** @brief Scaled physical values [x, y, z] (m/s^2 or rad/s). */
    std::array<double, 3> values{{0.0, 0.0, 0.0}};
};

/**
 * @class autodriver::hardware::CanImuDriver
 * @brief Fuses accel/gyro from two CAN frames via CanSensorDriverBase.
 *
 * Params: `interface`, `accel_can_id`, `gyro_can_id`, `accel_scale`,
 * `gyro_scale`.
 */
class CanImuDriver : public CanSensorDriverBase<CanImuDriver, ImuCanEvent> {
public:
    /**
     * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
     */
    AUTOLINK_SHARED_PTR_DEFINITIONS(CanImuDriver)

    /**
     * @brief Disable copy construction and copy assignment.
     */
    DISALLOW_COPY_AND_ASSIGN(CanImuDriver)
    /**
     * @brief Register accel/gyro ProtocolData and publish callback.
     * @param[in] id Sensor instance id.
     * @param[in] params DriverParams (cold-path parse only).
     */
    CanImuDriver(SensorId id, DriverParams params);

    /**
     * @brief Stops the CAN receive loop.
     */
    ~CanImuDriver() override { Stop(); }

    /**
     * @brief Report sensor type.
     * @return SensorType::kImu.
     */
    SensorType GetSensorType() const override { return SensorType::kImu; }

private:
    /**
     * @brief Merge accel/gyro halves; emit ImuSample when both present.
     * @param[in] event Decoded frame from MessageManager.
     */
    void OnEvent(const ImuCanEvent& event);

    /**
     * @brief Emit ImuSample if both halves and a callback are available.
     */
    void TryEmit();

    // CAN ids for accel / gyro frames.
    std::uint32_t accel_can_id_{0};
    std::uint32_t gyro_can_id_{0};

    // Raw int16 → physical scale factors.
    double accel_scale_{0.001};
    double gyro_scale_{0.0001};

    // Latest fused halves.
    std::array<double, 3> accel_{{0.0, 0.0, 0.0}};
    std::array<double, 3> gyro_{{0.0, 0.0, 0.0}};
    bool have_accel_{false};
    bool have_gyro_{false};
};

/**
 * @brief Factory for ImuBackendRegistry (REGISTER_IMU_BACKEND "can").
 * @param[in] id Sensor instance id from YAML.
 * @param[in] params Backend-specific key/value map.
 * @return Owning CanImuDriver* (never null).
 */
SensorDriver* CreateCanImuDriver(const SensorId& id,
                                 const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_IMU_CAN_IMU_DRIVER_HPP_

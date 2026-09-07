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
 * @brief SocketCAN IMU driver (scaled int16 triplet frames).
 */

#ifndef AUTODRIVER_IMU_CAN_IMU_DRIVER_HPP_
#define AUTODRIVER_IMU_CAN_IMU_DRIVER_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "autodriver/canbus/can_receiver.hpp"
#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_driver.hpp"

namespace autodriver {
namespace hardware {

/**
 * @brief Partial accel/gyro event from one CAN frame (fused in driver).
 */
struct ImuCanEvent {
    enum class Kind : std::uint8_t { kAccel, kGyro };
    Kind kind = Kind::kAccel;
    // Scaled physical values [x, y, z] (m/s^2 or rad/s).
    std::array<double, 3> values{{0.0, 0.0, 0.0}};
};

/**
 * @class autodriver::hardware::CanImuDriver
 * @brief Fuses accel/gyro from two CAN frames via CanReceiver + MessageManager.
 *
 * Params: `interface`, `accel_can_id`, `gyro_can_id`, `accel_scale`,
 * `gyro_scale`.
 */
class CanImuDriver : public SensorDriver {
public:
    CanImuDriver(SensorId id, DriverParams params);
    ~CanImuDriver() override;

    SensorType GetType() const override { return SensorType::kImu; }
    const SensorId& GetSensorId() const override { return id_; }

    bool Start() override;
    void Stop() override;
    bool IsRunning() const override;
    void SetSampleCallback(SampleCallback callback) override;

private:
    /** Merge accel/gyro halves; emit Imu when both present. */
    void OnEvent(const ImuCanEvent& event);
    void TryEmit();

    SensorId id_;
    DriverParams params_;
    // CAN ids for accel / gyro frames.
    std::uint32_t accel_can_id_{0};
    std::uint32_t gyro_can_id_{0};
    // Raw int16 → physical scale factors.
    double accel_scale_{0.001};
    double gyro_scale_{0.0001};

    canbus::CanReceiver<ImuCanEvent> receiver_;
    SampleCallback callback_;
    // Latest fused halves.
    std::array<double, 3> accel_{{0.0, 0.0, 0.0}};
    std::array<double, 3> gyro_{{0.0, 0.0, 0.0}};
    bool have_accel_{false};
    bool have_gyro_{false};
};

/**
 * @brief Factory for CanImuDriver (used by ImuModule backend "can").
 */
std::shared_ptr<SensorDriver> CreateCanImuDriver(const SensorId& id,
                                                 const DriverParams& params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_IMU_CAN_IMU_DRIVER_HPP_

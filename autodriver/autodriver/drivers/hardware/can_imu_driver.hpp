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

#ifndef AUTODRIVER_DRIVERS_HARDWARE_CAN_IMU_DRIVER_HPP_
#define AUTODRIVER_DRIVERS_HARDWARE_CAN_IMU_DRIVER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "autodriver/drivers/hardware/driver_params.hpp"
#include "autodriver/hal/sensor_driver.hpp"
#include "autodriver/io/can_socket.hpp"

namespace autodriver {
namespace hardware {

/**
 * @class CanImuDriver
 * @brief Fuses accel/gyro from two CAN frames with int16×3 layout.
 *
 * Params:
 * - interface (can0)
 * - accel_can_id, gyro_can_id (default 0x100 / 0x101)
 * - accel_scale (m/s^2 per LSB), gyro_scale (rad/s per LSB)
 */
class CanImuDriver : public SensorDriver
{
public:
  CanImuDriver(SensorId sensor_id, DriverParams params);
  ~CanImuDriver() override;

  SensorType GetType() const override { return SensorType::kImu; }
  const SensorId & GetSensorId() const override { return sensor_id_; }
  bool Start() override;
  void Stop() override;
  bool IsRunning() const override;
  void SetSampleCallback(SampleCallback callback) override;

private:
  void ReadLoop();
  void TryEmit();

  SensorId sensor_id_;
  DriverParams params_;
  std::uint32_t accel_can_id_{0};
  std::uint32_t gyro_can_id_{0};
  double accel_scale_{0.001};
  double gyro_scale_{0.0001};
  io::CanSocket socket_;
  SampleCallback callback_;
  std::array<double, 3> accel_{{0.0, 0.0, 0.0}};
  std::array<double, 3> gyro_{{0.0, 0.0, 0.0}};
  bool have_accel_{false};
  bool have_gyro_{false};
  std::atomic<bool> running_{false};
  std::thread worker_;
};

std::shared_ptr<SensorDriver> CreateCanImuDriver(
  const SensorId & sensor_id,
  const DriverParams & params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_DRIVERS_HARDWARE_CAN_IMU_DRIVER_HPP_

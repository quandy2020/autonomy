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

#ifndef AUTODRIVER_HARDWARE_CAN_IMU_DRIVER_HPP_
#define AUTODRIVER_HARDWARE_CAN_IMU_DRIVER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/hardware/can_socket.hpp"

namespace autodriver {
namespace hardware {

/**
 * @class autodriver::hardware::CanImuDriver
 * @brief Fuses accel/gyro from two CAN frames with int16×3 layout.
 * Params:
 * - interface (can0)
 * - accel_can_id, gyro_can_id (default 0x100 / 0x101)
 * - accel_scale (m/s^2 per LSB), gyro_scale (rad/s per LSB)
 */
class CanImuDriver : public SensorDriver
{
public:
  /**
   * @brief Constructor for autodriver::hardware::CanImuDriver
   * @param id Sensor identifier
   * @param params Driver configuration parameters
   */
  CanImuDriver(SensorId id, DriverParams params);

  /**
   * @brief Destructor for autodriver::hardware::CanImuDriver
   */
  ~CanImuDriver() override;

  /**
   * @brief Report sensor type
   * @return SensorType::kImu
   */
  SensorType GetType() const override { return SensorType::kImu; }

  /**
   * @brief Return this driver's sensor identifier
   * @return Sensor id assigned at construction
   */
  const SensorId & GetSensorId() const override { return id_; }

  /**
   * @brief Open the CAN socket and start the read thread
   * @return True on success
   */
  bool Start() override;

  /**
   * @brief Stop the read thread and close the CAN socket
   */
  void Stop() override;

  /**
   * @brief Whether the driver is actively reading
   * @return True while running
   */
  bool IsRunning() const override;

  /**
   * @brief Register callback invoked for each fused IMU sample
   * @param callback Sample delivery callback
   */
  void SetSampleCallback(SampleCallback callback) override;

private:
  /**
   * @brief Background loop that reads CAN frames and updates IMU state
   */
  void ReadLoop();

  /**
   * @brief Emit a sample when both accel and gyro frames have been received
   */
  void TryEmit();

  // Sensor identifier for this driver instance.
  SensorId id_;

  // Parsed driver parameters from configuration.
  DriverParams params_;

  // CAN identifier for accelerometer frames.
  std::uint32_t accel_can_id_{0};

  // CAN identifier for gyroscope frames.
  std::uint32_t gyro_can_id_{0};

  // Accelerometer scale factor in m/s^2 per LSB.
  double accel_scale_{0.001};

  // Gyroscope scale factor in rad/s per LSB.
  double gyro_scale_{0.0001};

  // SocketCAN receiver bound to the configured interface.
  io::CanSocket socket_;

  // User callback for delivered IMU samples.
  SampleCallback callback_;

  // Latest linear acceleration vector (m/s^2).
  std::array<double, 3> accel_{{0.0, 0.0, 0.0}};

  // Latest angular velocity vector (rad/s).
  std::array<double, 3> gyro_{{0.0, 0.0, 0.0}};

  // True after at least one accel frame was decoded.
  bool have_accel_{false};

  // True after at least one gyro frame was decoded.
  bool have_gyro_{false};

  // True while Start() succeeded and Stop() has not been called.
  std::atomic<bool> running_{false};

  // Worker thread running ReadLoop().
  std::thread worker_;
};

/**
 * @brief Factory used by ImuModule.
 * @param id Sensor identifier
 * @param params Driver configuration parameters
 * @return Shared sensor driver instance
 */
std::shared_ptr<SensorDriver> CreateCanImuDriver(
  const SensorId & id,
  const DriverParams & params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_HARDWARE_CAN_IMU_DRIVER_HPP_

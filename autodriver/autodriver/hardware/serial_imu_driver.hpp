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

#ifndef AUTODRIVER_HARDWARE_SERIAL_IMU_DRIVER_HPP_
#define AUTODRIVER_HARDWARE_SERIAL_IMU_DRIVER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/hardware/serial_port.hpp"
#include "autodriver/hardware/wit_motion_parser.hpp"

namespace autodriver {
namespace hardware {

/**
 * @class autodriver::hardware::SerialImuDriver
 * @brief Parses WIT-motion 0x55 binary protocol from a serial IMU.
 */
class SerialImuDriver : public SensorDriver
{
public:
  /**
   * @brief Constructor for autodriver::hardware::SerialImuDriver
   * @param id Sensor identifier
   * @param params Driver configuration parameters
   */
  SerialImuDriver(SensorId id, DriverParams params);

  /**
   * @brief Destructor for autodriver::hardware::SerialImuDriver
   */
  ~SerialImuDriver() override;

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
   * @brief Open the serial port and start the read thread
   * @return True on success
   */
  bool Start() override;

  /**
   * @brief Stop the read thread and close the serial port
   */
  void Stop() override;

  /**
   * @brief Whether the driver is actively reading
   * @return True while running
   */
  bool IsRunning() const override;

  /**
   * @brief Register callback invoked for each complete IMU sample
   * @param callback Sample delivery callback
   */
  void SetSampleCallback(SampleCallback callback) override;

private:
  /**
   * @brief Background loop that reads bytes and emits fused IMU samples
   */
  void ReadLoop();

  // Sensor identifier for this driver instance.
  SensorId id_;

  // Parsed driver parameters from configuration.
  DriverParams params_;

  // Serial port connected to the IMU module.
  io::SerialPort port_;

  // Incremental WIT-motion protocol parser.
  protocol::WitMotionParser parser_;

  // User callback for delivered IMU samples.
  SampleCallback callback_;

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
std::shared_ptr<SensorDriver> CreateSerialImuDriver(
  const SensorId & id,
  const DriverParams & params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_HARDWARE_SERIAL_IMU_DRIVER_HPP_

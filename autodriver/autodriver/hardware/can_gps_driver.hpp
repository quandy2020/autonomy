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
 * @brief SocketCAN GNSS driver (NMEA2000 lat/lon frame).
 */

#ifndef AUTODRIVER_HARDWARE_CAN_GPS_DRIVER_HPP_
#define AUTODRIVER_HARDWARE_CAN_GPS_DRIVER_HPP_

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
 * @class autodriver::hardware::CanGpsDriver
 * @brief Decodes lat/lon from a CAN frame (NMEA2000 PGN 129025 layout).
 * Params: interface (can0), can_id (default 0x12902500), format (nmea2000_latlon).
 */
class CanGpsDriver : public SensorDriver
{
public:
  /**
   * @brief Stores sensor identity and the target CAN frame id for GPS fixes.
   */
  CanGpsDriver(SensorId id, DriverParams params);

  /**
   * @brief Stops the reader thread and closes the CAN socket.
   */
  ~CanGpsDriver() override;

  /**
   * @brief Report sensor type
   * @return SensorType::kGps
   */
  SensorType GetType() const override { return SensorType::kGps; }

  /**
   * @brief Return this driver's sensor identifier
   * @return Sensor id assigned at construction
   */
  const SensorId & GetSensorId() const override { return id_; }

  /**
   * @brief Opens the CAN interface and starts the frame reader thread.
   */
  bool Start() override;

  /**
   * @brief Stops reading and joins the worker thread.
   */
  void Stop() override;

  /**
   * @brief Returns true while the CAN reader thread is active.
   */
  bool IsRunning() const override;

  /**
   * @brief Registers the callback invoked for each GPS fix sample.
   */
  void SetSampleCallback(SampleCallback callback) override;

private:
  /**
   * @brief Reads CAN frames and emits GPS samples for matching NMEA2000 ids.
   */
  void ReadLoop();

  // Sensor identifier for this driver instance.
  SensorId id_;

  // Parsed driver parameters from configuration.
  DriverParams params_;

  // Expected CAN frame identifier for lat/lon messages.
  std::uint32_t can_id_{0};

  // SocketCAN receiver bound to the configured interface.
  io::CanSocket socket_;

  // User callback for delivered GPS samples.
  SampleCallback callback_;

  // True while Start() succeeded and Stop() has not been called.
  std::atomic<bool> running_{false};

  // Worker thread running ReadLoop().
  std::thread worker_;
};

/**
 * @brief Factory used by GpsModule.
 */
std::shared_ptr<SensorDriver> CreateCanGpsDriver(
  const SensorId & id,
  const DriverParams & params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_HARDWARE_CAN_GPS_DRIVER_HPP_

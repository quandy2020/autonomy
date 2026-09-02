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
   * @brief Constructor for autodriver::hardware::CanGpsDriver
   * @param id Sensor identifier
   * @param params Driver configuration parameters
   */
  CanGpsDriver(SensorId id, DriverParams params);

  /**
   * @brief Destructor for autodriver::hardware::CanGpsDriver
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
   * @brief Register callback invoked for each decoded GPS fix
   * @param callback Sample delivery callback
   */
  void SetSampleCallback(SampleCallback callback) override;

private:
  /**
   * @brief Background loop that reads CAN frames and emits GPS fixes
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
 * @param id Sensor identifier
 * @param params Driver configuration parameters
 * @return Shared sensor driver instance
 */
std::shared_ptr<SensorDriver> CreateCanGpsDriver(
  const SensorId & id,
  const DriverParams & params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_HARDWARE_CAN_GPS_DRIVER_HPP_

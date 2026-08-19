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
 * @class CanGpsDriver
 * @brief Decodes lat/lon from a CAN frame (NMEA2000 PGN 129025 layout).
 *
 * Params: interface (can0), can_id (default 0x12902500), format (nmea2000_latlon).
 */
class CanGpsDriver : public SensorDriver
{
public:
  CanGpsDriver(SensorId id, DriverParams params);
  ~CanGpsDriver() override;

  SensorType GetType() const override { return SensorType::kGps; }
  const SensorId & GetSensorId() const override { return id_; }
  bool Start() override;
  void Stop() override;
  bool IsRunning() const override;
  void SetSampleCallback(SampleCallback callback) override;

private:
  void ReadLoop();

  SensorId id_;
  DriverParams params_;
  std::uint32_t can_id_{0};
  io::CanSocket socket_;
  SampleCallback callback_;
  std::atomic<bool> running_{false};
  std::thread worker_;
};

std::shared_ptr<SensorDriver> CreateCanGpsDriver(
  const SensorId & id,
  const DriverParams & params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_HARDWARE_CAN_GPS_DRIVER_HPP_

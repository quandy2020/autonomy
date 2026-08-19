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
 * @brief Serial NMEA GNSS receiver driver.
 */

#ifndef AUTODRIVER_HARDWARE_SERIAL_GPS_DRIVER_HPP_
#define AUTODRIVER_HARDWARE_SERIAL_GPS_DRIVER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/hardware/serial_port.hpp"

namespace autodriver {
namespace hardware {

/**
 * @class SerialGpsDriver
 * @brief Reads NMEA 0183 GGA/RMC sentences from a serial GNSS module.
 *
 * Required params: device (/dev/ttyUSB0). Optional: baud (115200).
 */
class SerialGpsDriver : public SensorDriver
{
public:
  SerialGpsDriver(SensorId id, DriverParams params);
  ~SerialGpsDriver() override;

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
  io::SerialPort port_;
  SampleCallback callback_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  std::string line_buffer_;
};

std::shared_ptr<SensorDriver> CreateSerialGpsDriver(
  const SensorId & id,
  const DriverParams & params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_HARDWARE_SERIAL_GPS_DRIVER_HPP_

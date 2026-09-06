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

#ifndef AUTODRIVER_GPS_SERIAL_GPS_DRIVER_HPP_
#define AUTODRIVER_GPS_SERIAL_GPS_DRIVER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "autodriver/common/stream.hpp"
#include "autodriver/driver_params.hpp"
#include "autodriver/gps/parser/parser.hpp"
#include "autodriver/sensor_driver.hpp"

namespace autodriver {
namespace hardware {

/**
 * @class autodriver::hardware::SerialGpsDriver
 * @brief Reads NMEA via common::Stream + gps::GnssParser ("nmea").
 * Required params: device (/dev/ttyUSB0). Optional: baud (115200).
 */
class SerialGpsDriver : public SensorDriver
{
public:
  /**
   * @brief Stores sensor identity and serial driver params.
   */
  SerialGpsDriver(SensorId id, DriverParams params);

  /**
   * @brief Stops the reader thread and closes the serial port.
   */
  ~SerialGpsDriver() override;

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
   * @brief Opens the serial device and starts the NMEA reader thread.
   */
  bool Start() override;

  /**
   * @brief Stops reading and joins the worker thread.
   */
  void Stop() override;

  /**
   * @brief Returns true while the serial reader thread is active.
   */
  bool IsRunning() const override;

  /**
   * @brief Registers the callback invoked for each GPS fix sample.
   */
  void SetSampleCallback(SampleCallback callback) override;

private:
  /**
   * @brief Reads NMEA lines from serial and emits parsed GPS samples.
   */
  void ReadLoop();

  // Sensor identifier for this driver instance.
  SensorId id_;

  // Parsed driver parameters from configuration.
  DriverParams params_;

  // Serial (or future TCP/UDP) transport to the GNSS module.
  std::unique_ptr<common::Stream> stream_;

  // User callback for delivered GPS samples.
  SampleCallback callback_;

  // True while Start() succeeded and Stop() has not been called.
  std::atomic<bool> running_{false};

  // Worker thread running ReadLoop().
  std::thread worker_;

  // Streaming NMEA parser (line buffer lives inside the parser).
  std::unique_ptr<gps::GnssParser> parser_;
};

/**
 * @brief Factory used by GpsModule.
 */
std::shared_ptr<SensorDriver> CreateSerialGpsDriver(
  const SensorId & id,
  const DriverParams & params);

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_GPS_SERIAL_GPS_DRIVER_HPP_

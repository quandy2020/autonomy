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
 * @brief Implements SerialGpsDriver.
 */

#include "autodriver/hardware/serial_gps_driver.hpp"

#include <utility>

#include "autolink/time/time.hpp"
#include "autodriver/hardware/nmea_0183.hpp"
#include "autodriver/types/sensor_sample.hpp"

namespace autodriver {
namespace hardware {

/**
 * @brief Stores sensor identity and serial driver params.
 */
SerialGpsDriver::SerialGpsDriver(SensorId id, DriverParams params)
: id_(std::move(id)),
  params_(std::move(params))
{
}

/**
 * @brief Stops the reader thread and closes the serial port.
 */
SerialGpsDriver::~SerialGpsDriver()
{
  Stop();
}

/**
 * @brief Opens the serial device and starts the NMEA reader thread.
 */
bool SerialGpsDriver::Start()
{
  if (running_.exchange(true)) {
    return true;
  }

  const std::string device = GetString(params_, "device", "/dev/ttyUSB0");
  const int baud = ParseInt(params_, "baud", 115200);
  if (!port_.Open(device, baud)) {
    running_ = false;
    return false;
  }

  worker_ = std::thread([this]() { ReadLoop(); });
  return true;
}

/**
 * @brief Stops reading and joins the worker thread.
 */
void SerialGpsDriver::Stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  port_.Close();
  if (worker_.joinable()) {
    worker_.join();
  }
}

/**
 * @brief Returns true while the serial reader thread is active.
 */
bool SerialGpsDriver::IsRunning() const
{
  return running_.load();
}

/**
 * @brief Registers the callback invoked for each GPS fix sample.
 */
void SerialGpsDriver::SetSampleCallback(SampleCallback callback)
{
  callback_ = std::move(callback);
}

/**
 * @brief Reads NMEA lines from serial and emits parsed GPS samples.
 */
void SerialGpsDriver::ReadLoop()
{
  std::uint8_t chunk[256];
  while (running_.load()) {
    const std::size_t n = port_.Read(chunk, sizeof(chunk), 100);
    if (n == 0) {
      continue;
    }

    line_buffer_.append(reinterpret_cast<char *>(chunk), n);
    std::size_t pos = 0;
    while ((pos = line_buffer_.find('\n')) != std::string::npos) {
      std::string line = line_buffer_.substr(0, pos);
      line_buffer_.erase(0, pos + 1);
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      if (line.empty() || line[0] != '$') {
        continue;
      }

      std::optional<protocol::NmeaGgaFix> fix;
      if (line.find("GGA") != std::string::npos) {
        fix = protocol::ParseGgaSentence(line);
      } else if (line.find("RMC") != std::string::npos) {
        fix = protocol::ParseRmcSentence(line);
      }
      if (!fix || !callback_) {
        continue;
      }

      callback_(std::make_unique<GpsSample>(
          id_, autolink::Time::Now(),
          GpsMsg(fix->latitude_deg, fix->longitude_deg, fix->altitude_m,
                 fix->status)));
    }
  }
}

/**
 * @brief Factory that constructs a SerialGpsDriver instance.
 */
std::shared_ptr<SensorDriver> CreateSerialGpsDriver(
  const SensorId & id,
  const DriverParams & params)
{
  return std::make_shared<SerialGpsDriver>(id, params);
}

}  // namespace hardware
}  // namespace autodriver

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
 * @brief Implements SerialImuDriver.
 */

#include "autodriver/hardware/serial_imu_driver.hpp"

#include <utility>

#include "autolink/time/time.hpp"
#include "autodriver/types/sensor_sample.hpp"

namespace autodriver {
namespace hardware {

/** @brief Stores sensor identity and WIT-motion parser scale factors. */
SerialImuDriver::SerialImuDriver(SensorId id, DriverParams params)
: id_(std::move(id)),
  params_(std::move(params)),
  parser_(
    ParseDouble(params_, "accel_scale", 16.0 * 9.80665 / 32768.0),
    ParseDouble(
      params_, "gyro_scale",
      2000.0 * 3.141592653589793 / 180.0 / 32768.0))
{
}

/** @brief Stops the reader thread and closes the serial port. */
SerialImuDriver::~SerialImuDriver()
{
  Stop();
}

/** @brief Opens the serial device and starts the byte reader thread. */
bool SerialImuDriver::Start()
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

/** @brief Stops reading and joins the worker thread. */
void SerialImuDriver::Stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  port_.Close();
  if (worker_.joinable()) {
    worker_.join();
  }
}

/** @brief Returns true while the serial reader thread is active. */
bool SerialImuDriver::IsRunning() const
{
  return running_.load();
}

/** @brief Registers the callback invoked for each fused IMU sample. */
void SerialImuDriver::SetSampleCallback(SampleCallback callback)
{
  callback_ = std::move(callback);
}

/** @brief Reads serial bytes, parses WIT packets, and emits IMU samples. */
void SerialImuDriver::ReadLoop()
{
  std::uint8_t chunk[256];
  while (running_.load()) {
    const std::size_t n = port_.Read(chunk, sizeof(chunk), 50);
    for (std::size_t i = 0; i < n; ++i) {
      parser_.Feed(chunk[i]);
      if (!parser_.HasCompleteSample() || !callback_) {
        continue;
      }

      const auto & state = parser_.state();
      callback_(std::make_unique<ImuSample>(
          id_, autolink::Time::Now(),
          ImuMsg(state.angular_velocity, state.linear_acceleration)));
      parser_.ResetSampleFlags();
    }
  }
}

/** @brief Factory that constructs a SerialImuDriver instance. */
std::shared_ptr<SensorDriver> CreateSerialImuDriver(
  const SensorId & id,
  const DriverParams & params)
{
  return std::make_shared<SerialImuDriver>(id, params);
}

}  // namespace hardware
}  // namespace autodriver

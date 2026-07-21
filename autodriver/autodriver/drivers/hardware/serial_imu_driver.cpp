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

#include "autodriver/drivers/hardware/serial_imu_driver.hpp"

#include <utility>

#include "autodriver/common/time.hpp"
#include "autodriver/types/imu_sample.hpp"

namespace autodriver {
namespace hardware {

SerialImuDriver::SerialImuDriver(SensorId sensor_id, DriverParams params)
: sensor_id_(std::move(sensor_id)),
  params_(std::move(params)),
  parser_(
    ParseDouble(params_, "accel_scale", 16.0 * 9.80665 / 32768.0),
    ParseDouble(
      params_, "gyro_scale",
      2000.0 * 3.141592653589793 / 180.0 / 32768.0))
{
}

SerialImuDriver::~SerialImuDriver()
{
  Stop();
}

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

bool SerialImuDriver::IsRunning() const
{
  return running_.load();
}

void SerialImuDriver::SetSampleCallback(SampleCallback callback)
{
  callback_ = std::move(callback);
}

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
          sensor_id_,
          Now(),
          state.linear_acceleration,
          state.angular_velocity));
      parser_.ResetSampleFlags();
    }
  }
}

std::shared_ptr<SensorDriver> CreateSerialImuDriver(
  const SensorId & sensor_id,
  const DriverParams & params)
{
  return std::make_shared<SerialImuDriver>(sensor_id, params);
}

}  // namespace hardware
}  // namespace autodriver

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
 * @brief Implements CanImuDriver.
 */

#include "autodriver/drivers/hardware/can_imu_driver.hpp"

#include <utility>

#include "autodriver/common/time.hpp"
#include "autodriver/protocol/wit_motion.hpp"
#include "autodriver/types/imu_sample.hpp"

namespace autodriver {
namespace hardware {

CanImuDriver::CanImuDriver(SensorId sensor_id, DriverParams params)
: sensor_id_(std::move(sensor_id)),
  params_(std::move(params)),
  accel_can_id_(ParseCanId(params_, "accel_can_id", 0x100)),
  gyro_can_id_(ParseCanId(params_, "gyro_can_id", 0x101)),
  accel_scale_(ParseDouble(params_, "accel_scale", 0.001)),
  gyro_scale_(ParseDouble(params_, "gyro_scale", 0.0001))
{
}

CanImuDriver::~CanImuDriver()
{
  Stop();
}

bool CanImuDriver::Start()
{
  if (running_.exchange(true)) {
    return true;
  }

  const std::string interface_name = GetString(params_, "interface", "can0");
  if (!socket_.Open(interface_name)) {
    running_ = false;
    return false;
  }

  worker_ = std::thread([this]() { ReadLoop(); });
  return true;
}

void CanImuDriver::Stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  socket_.Close();
  if (worker_.joinable()) {
    worker_.join();
  }
}

bool CanImuDriver::IsRunning() const
{
  return running_.load();
}

void CanImuDriver::SetSampleCallback(SampleCallback callback)
{
  callback_ = std::move(callback);
}

void CanImuDriver::TryEmit()
{
  if (!have_accel_ || !have_gyro_ || !callback_) {
    return;
  }
  callback_(std::make_unique<ImuSample>(
      sensor_id_, Now(), gyro_, accel_));
  have_accel_ = false;
  have_gyro_ = false;
}

void CanImuDriver::ReadLoop()
{
  while (running_.load()) {
    io::CanFrame frame;
    if (!socket_.Read(frame, 100)) {
      continue;
    }

    const std::uint32_t id = frame.extended ? (frame.id & 0x1FFFFFFF) : frame.id;
    if (id == accel_can_id_ && frame.dlc >= 6) {
      accel_ = protocol::DecodeScaledInt16Triplet(frame.data, accel_scale_);
      have_accel_ = true;
      TryEmit();
    } else if (id == gyro_can_id_ && frame.dlc >= 6) {
      gyro_ = protocol::DecodeScaledInt16Triplet(frame.data, gyro_scale_);
      have_gyro_ = true;
      TryEmit();
    }
  }
}

std::shared_ptr<SensorDriver> CreateCanImuDriver(
  const SensorId & sensor_id,
  const DriverParams & params)
{
  return std::make_shared<CanImuDriver>(sensor_id, params);
}

}  // namespace hardware
}  // namespace autodriver

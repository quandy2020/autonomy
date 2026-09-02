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

#include "autodriver/hardware/can_imu_driver.hpp"

#include <utility>

#include "autolink/time/time.hpp"
#include "autodriver/hardware/wit_motion_parser.hpp"
#include "autodriver/types/sensor_sample.hpp"

namespace autodriver {
namespace hardware {

/**
 * @brief Stores sensor identity and CAN ids/scales for accel and gyro frames.
 */
CanImuDriver::CanImuDriver(SensorId id, DriverParams params)
: id_(std::move(id)),
  params_(std::move(params)),
  accel_can_id_(ParseCanId(params_, "accel_can_id", 0x100)),
  gyro_can_id_(ParseCanId(params_, "gyro_can_id", 0x101)),
  accel_scale_(ParseDouble(params_, "accel_scale", 0.001)),
  gyro_scale_(ParseDouble(params_, "gyro_scale", 0.0001))
{
}

/**
 * @brief Stops the reader thread and closes the CAN socket.
 */
CanImuDriver::~CanImuDriver()
{
  Stop();
}

/**
 * @brief Opens the CAN interface and starts the frame reader thread.
 */
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

/**
 * @brief Stops reading and joins the worker thread.
 */
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

/**
 * @brief Returns true while the CAN reader thread is active.
 */
bool CanImuDriver::IsRunning() const
{
  return running_.load();
}

/**
 * @brief Registers the callback invoked for each fused IMU sample.
 */
void CanImuDriver::SetSampleCallback(SampleCallback callback)
{
  callback_ = std::move(callback);
}

/**
 * @brief Emits an IMU sample when both accel and gyro frames have arrived.
 */
void CanImuDriver::TryEmit()
{
  if (!have_accel_ || !have_gyro_ || !callback_) {
    return;
  }
  callback_(std::make_unique<ImuSample>(
      id_, autolink::Time::Now(), ImuMsg(gyro_, accel_)));
  have_accel_ = false;
  have_gyro_ = false;
}

/**
 * @brief Reads CAN frames and updates partial accel/gyro state by frame id.
 */
void CanImuDriver::ReadLoop()
{
  while (running_.load()) {
    io::CanFrame frame;
    if (!socket_.Read(frame, 100)) {
      continue;
    }

    // Normalized CAN id with extended-frame flag stripped.
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

/**
 * @brief Factory that constructs a CanImuDriver instance.
 */
std::shared_ptr<SensorDriver> CreateCanImuDriver(
  const SensorId & id,
  const DriverParams & params)
{
  return std::make_shared<CanImuDriver>(id, params);
}

}  // namespace hardware
}  // namespace autodriver

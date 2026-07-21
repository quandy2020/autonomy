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
 * @brief Implements CanGpsDriver.
 */

#include "autodriver/drivers/hardware/can_gps_driver.hpp"

#include <utility>

#include "autodriver/common/time.hpp"
#include "autodriver/protocol/wit_motion.hpp"
#include "autodriver/types/gps_sample.hpp"

namespace autodriver {
namespace hardware {

CanGpsDriver::CanGpsDriver(SensorId sensor_id, DriverParams params)
: sensor_id_(std::move(sensor_id)),
  params_(std::move(params)),
  can_id_(ParseCanId(params_, "can_id", 0x12902500))
{
}

CanGpsDriver::~CanGpsDriver()
{
  Stop();
}

bool CanGpsDriver::Start()
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

void CanGpsDriver::Stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  socket_.Close();
  if (worker_.joinable()) {
    worker_.join();
  }
}

bool CanGpsDriver::IsRunning() const
{
  return running_.load();
}

void CanGpsDriver::SetSampleCallback(SampleCallback callback)
{
  callback_ = std::move(callback);
}

void CanGpsDriver::ReadLoop()
{
  while (running_.load()) {
    io::CanFrame frame;
    if (!socket_.Read(frame, 100)) {
      continue;
    }

    const std::uint32_t match_id = frame.extended ? (frame.id & 0x1FFFFFFF) : frame.id;
    if (match_id != can_id_) {
      continue;
    }

    const auto fix = protocol::ParseNmea2000LatLonFrame(frame.data, frame.dlc);
    if (!fix || !callback_) {
      continue;
    }

    callback_(std::make_unique<GpsSample>(
        sensor_id_,
        Now(),
        fix->latitude_deg,
        fix->longitude_deg,
        0.0,
        GpsFixStatus::kFix3D));
  }
}

std::shared_ptr<SensorDriver> CreateCanGpsDriver(
  const SensorId & sensor_id,
  const DriverParams & params)
{
  return std::make_shared<CanGpsDriver>(sensor_id, params);
}

}  // namespace hardware
}  // namespace autodriver

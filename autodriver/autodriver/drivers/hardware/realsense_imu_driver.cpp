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
 * @brief Implements RealSenseImuDriver.
 */

#include "autodriver/drivers/hardware/realsense_imu_driver.hpp"

#include <utility>

#include "autodriver/common/time.hpp"
#include "autodriver/types/imu_sample.hpp"

namespace autodriver {
namespace hardware {

RealSenseImuDriver::RealSenseImuDriver(
  SensorId sensor_id,
  DriverParams params)
: sensor_id_(std::move(sensor_id)),
  params_(std::move(params))
{
}

RealSenseImuDriver::~RealSenseImuDriver()
{
  Stop();
}

bool RealSenseImuDriver::Start()
{
  if (running_.exchange(true)) {
    return true;
  }

  hub_ = io::RealSenseDeviceHub::Acquire(params_);
  const SensorId sensor_id = sensor_id_;
  SampleCallback callback = callback_;
  subscription_id_ = hub_->SubscribeImu(
    [sensor_id, callback](
      const std::array<double, 3> & accel,
      const std::array<double, 3> & gyro) {
      if (!callback) {
        return;
      }
      callback(std::make_unique<ImuSample>(
          sensor_id, Now(), gyro, accel));
    });

  if (!hub_->IsRunning() && !hub_->Start()) {
    hub_->Unsubscribe(subscription_id_);
    subscription_id_ = 0;
    running_ = false;
    return false;
  }

  return true;
}

void RealSenseImuDriver::Stop()
{
  if (!running_.exchange(false)) {
    return;
  }

  if (hub_ && subscription_id_ != 0) {
    hub_->Unsubscribe(subscription_id_);
    subscription_id_ = 0;
  }
  hub_.reset();
}

bool RealSenseImuDriver::IsRunning() const
{
  return running_.load();
}

void RealSenseImuDriver::SetSampleCallback(SampleCallback callback)
{
  callback_ = std::move(callback);
}

std::shared_ptr<SensorDriver> CreateRealSenseImuDriver(
  const SensorId & sensor_id,
  const DriverParams & params)
{
  return std::make_shared<RealSenseImuDriver>(sensor_id, params);
}

}  // namespace hardware
}  // namespace autodriver

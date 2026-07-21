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
 * @brief Implements RealSenseCameraDriver.
 */

#include "autodriver/drivers/hardware/realsense_camera_driver.hpp"

#include <utility>

#include "autodriver/common/time.hpp"
#include "autodriver/types/camera_frame.hpp"

namespace autodriver {
namespace hardware {

RealSenseCameraDriver::RealSenseCameraDriver(
  SensorId sensor_id,
  DriverParams params)
: sensor_id_(std::move(sensor_id)),
  params_(std::move(params)),
  stream_(realsense::ParseStreamKind(
      GetString(params_, "stream"), realsense::StreamKind::kColor)),
  width_(ParseInt(params_, "width", 640)),
  height_(ParseInt(params_, "height", 480)),
  fps_(ParseInt(params_, "fps", 30))
{
}

RealSenseCameraDriver::~RealSenseCameraDriver()
{
  Stop();
}

bool RealSenseCameraDriver::Start()
{
  if (running_.exchange(true)) {
    return true;
  }

  hub_ = io::RealSenseDeviceHub::Acquire(params_);
  const SensorId sensor_id = sensor_id_;
  SampleCallback callback = callback_;
  subscription_id_ = hub_->SubscribeVideo(
    stream_, width_, height_, fps_,
    [sensor_id, callback](io::RealSenseVideoFrame frame) {
      if (!callback) {
        return;
      }
      callback(std::make_unique<CameraFrame>(
          sensor_id,
          Now(),
          frame.width,
          frame.height,
          frame.encoding,
          std::move(frame.data)));
    });

  if (!hub_->IsRunning() && !hub_->Start()) {
    hub_->Unsubscribe(subscription_id_);
    subscription_id_ = 0;
    running_ = false;
    return false;
  }

  return true;
}

void RealSenseCameraDriver::Stop()
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

bool RealSenseCameraDriver::IsRunning() const
{
  return running_.load();
}

void RealSenseCameraDriver::SetSampleCallback(SampleCallback callback)
{
  callback_ = std::move(callback);
}

std::shared_ptr<SensorDriver> CreateRealSenseCameraDriver(
  const SensorId & sensor_id,
  const DriverParams & params)
{
  return std::make_shared<RealSenseCameraDriver>(sensor_id, params);
}

}  // namespace hardware
}  // namespace autodriver

/*
 * Copyright 2026 Autodriver contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file
 * @brief RealSensePointCloudDriver implementation.
 */

#include "autodriver/hardware/realsense_pointcloud_driver.hpp"

#include <utility>

#include "autodriver/types/sensor_sample.hpp"
#include "autolink/time/time.hpp"

namespace autodriver {
namespace hardware {
namespace {

/** @brief Converts a RealSense timestamp in milliseconds to autolink::Time. */
autolink::Time TimeFromMilliseconds(const double ms) {
    return autolink::Time(static_cast<std::uint64_t>(ms * 1000000.0));
}

}  // namespace

/** @brief Parses resolution params and stores sensor identity. */
RealSensePointCloudDriver::RealSensePointCloudDriver(SensorId id,
                                                     DriverParams params)
    : id_(std::move(id)),
      params_(std::move(params)),
      width_(ParseInt(params_, "width", 640)),
      height_(ParseInt(params_, "height", 480)),
      fps_(ParseInt(params_, "fps", 30)) {}

/** @brief Unsubscribes and stops the shared device hub on destruction. */
RealSensePointCloudDriver::~RealSensePointCloudDriver() { Stop(); }

/** @brief Subscribes to a RealSense depth point cloud via the shared hub. */
bool RealSensePointCloudDriver::Start() {
    if (running_.exchange(true)) {
        return true;
    }

    hub_ = io::RealSenseDeviceHub::Acquire(params_);
    const SensorId id = id_;
    SampleCallback callback = callback_;
    subscription_id_ = hub_->SubscribePointCloud(
        width_, height_, fps_,
        [id, callback](io::RealSensePointCloudFrame frame) {
            if (!callback) {
                return;
            }
            auto sample = std::make_unique<LidarCloud>(
                id, TimeFromMilliseconds(frame.timestamp_ms),
                std::move(frame.cloud));
            sample->frame_id = frame.frame_id;
            callback(std::move(sample));
        });

    if (!hub_->IsRunning() && !hub_->Start()) {
        hub_->Unsubscribe(subscription_id_);
        subscription_id_ = 0;
        running_ = false;
        return false;
    }

    return true;
}

/** @brief Unsubscribes from the hub and releases the shared device handle. */
void RealSensePointCloudDriver::Stop() {
    if (!running_.exchange(false)) {
        return;
    }

    if (hub_ && subscription_id_ != 0) {
        hub_->Unsubscribe(subscription_id_);
        subscription_id_ = 0;
    }
    hub_.reset();
}

/** @brief Returns true while the point-cloud subscription is active. */
bool RealSensePointCloudDriver::IsRunning() const { return running_.load(); }

/** @brief Registers the callback invoked for each point-cloud sample. */
void RealSensePointCloudDriver::SetSampleCallback(SampleCallback callback) {
    callback_ = std::move(callback);
}

/** @brief Factory that constructs a RealSensePointCloudDriver instance. */
std::shared_ptr<SensorDriver> CreateRealSensePointCloudDriver(
    const SensorId& id, const DriverParams& params) {
    return std::make_shared<RealSensePointCloudDriver>(id, params);
}

}  // namespace hardware
}  // namespace autodriver

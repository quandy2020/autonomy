/*
 * Copyright 2026 Autodriver contributors
 */

#include "autodriver/hardware/realsense_pointcloud_driver.hpp"

#include <utility>

#include "autodriver/types/sensor_sample.hpp"
#include "autolink/time/time.hpp"

namespace autodriver {
namespace hardware {
namespace {

autolink::Time TimeFromMilliseconds(const double ms) {
    return autolink::Time(static_cast<std::uint64_t>(ms * 1000000.0));
}

}  // namespace

RealSensePointCloudDriver::RealSensePointCloudDriver(SensorId id,
                                                     DriverParams params)
    : id_(std::move(id)),
      params_(std::move(params)),
      width_(ParseInt(params_, "width", 640)),
      height_(ParseInt(params_, "height", 480)),
      fps_(ParseInt(params_, "fps", 30)) {}

RealSensePointCloudDriver::~RealSensePointCloudDriver() { Stop(); }

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

bool RealSensePointCloudDriver::IsRunning() const { return running_.load(); }

void RealSensePointCloudDriver::SetSampleCallback(SampleCallback callback) {
    callback_ = std::move(callback);
}

std::shared_ptr<SensorDriver> CreateRealSensePointCloudDriver(
    const SensorId& id, const DriverParams& params) {
    return std::make_shared<RealSensePointCloudDriver>(id, params);
}

}  // namespace hardware
}  // namespace autodriver

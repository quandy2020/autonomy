/*
 * Copyright 2026 Autodriver contributors
 */

#include "autodriver/hardware/realsense_imu_driver.hpp"

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

RealSenseImuDriver::RealSenseImuDriver(SensorId id, DriverParams params)
    : id_(std::move(id)), params_(std::move(params)) {}

RealSenseImuDriver::~RealSenseImuDriver() { Stop(); }

bool RealSenseImuDriver::Start() {
    if (running_.exchange(true)) {
        return true;
    }

    hub_ = io::RealSenseDeviceHub::Acquire(params_);
    const SensorId id = id_;
    const std::string frame_id =
        GetString(params_, "frame_id").empty()
            ? "camera_imu_optical_frame"
            : GetString(params_, "frame_id");
    SampleCallback callback = callback_;
    subscription_id_ = hub_->SubscribeImu(
        [id, frame_id, callback](const std::array<double, 3>& accel,
                                 const std::array<double, 3>& gyro,
                                 const double timestamp_ms) {
            if (!callback) {
                return;
            }
            auto sample = std::make_unique<ImuSample>(
                id, TimeFromMilliseconds(timestamp_ms),
                ImuMsg(gyro, accel, frame_id));
            sample->msg.mutable_header()->set_frame_id(frame_id);
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

void RealSenseImuDriver::Stop() {
    if (!running_.exchange(false)) {
        return;
    }

    if (hub_ && subscription_id_ != 0) {
        hub_->Unsubscribe(subscription_id_);
        subscription_id_ = 0;
    }
    hub_.reset();
}

bool RealSenseImuDriver::IsRunning() const { return running_.load(); }

void RealSenseImuDriver::SetSampleCallback(SampleCallback callback) {
    callback_ = std::move(callback);
}

std::shared_ptr<SensorDriver> CreateRealSenseImuDriver(const SensorId& id,
                                                       const DriverParams& params) {
    return std::make_shared<RealSenseImuDriver>(id, params);
}

}  // namespace hardware
}  // namespace autodriver

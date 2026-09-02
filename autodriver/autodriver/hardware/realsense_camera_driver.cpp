/*
 * Copyright 2026 Autodriver contributors
 */

#include "autodriver/hardware/realsense_camera_driver.hpp"

#include <utility>

#include "autodriver/types/sensor_sample.hpp"
#include "autolink/time/time.hpp"

namespace autodriver {
namespace hardware {
namespace {

autolink::Time TimeFromMilliseconds(const double ms) {
    return autolink::Time(static_cast<std::uint64_t>(ms * 1000000.0));
}

std::string ResolveFrameId(const DriverParams& params,
                           const hardware::realsense::StreamKind stream) {
    const std::string configured = GetString(params, "frame_id");
    if (!configured.empty()) {
        return configured;
    }
    return hardware::realsense::DefaultFrameId(stream);
}

}  // namespace

RealSenseCameraDriver::RealSenseCameraDriver(SensorId id, DriverParams params)
    : id_(std::move(id)),
      params_(std::move(params)),
      stream_(realsense::ParseStreamKind(GetString(params_, "stream"),
                                         realsense::StreamKind::kColor)),
      width_(ParseInt(params_, "width", 640)),
      height_(ParseInt(params_, "height", 480)),
      fps_(ParseInt(params_, "fps", 30)) {}

RealSenseCameraDriver::~RealSenseCameraDriver() { Stop(); }

bool RealSenseCameraDriver::Start() {
    if (running_.exchange(true)) {
        return true;
    }

    hub_ = io::RealSenseDeviceHub::Acquire(params_);
    const SensorId id = id_;
    const std::string frame_id = ResolveFrameId(params_, stream_);
    SampleCallback callback = callback_;
    subscription_id_ = hub_->SubscribeVideo(
        stream_, width_, height_, fps_,
        [id, frame_id, callback](io::RealSenseVideoFrame frame) {
            if (!callback) {
                return;
            }
            auto sample = std::make_unique<CameraFrame>(
                id, TimeFromMilliseconds(frame.timestamp_ms),
                ImageMsg(frame.width, frame.height, std::move(frame.encoding),
                         frame.data));
            sample->frame_id = frame_id.empty() ? frame.frame_id : frame_id;
            if (frame.has_camera_info) {
                sample->camera_info = frame.camera_info;
                sample->camera_info.mutable_header()->set_frame_id(
                    sample->frame_id);
                sample->has_camera_info = true;
            }
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

void RealSenseCameraDriver::Stop() {
    if (!running_.exchange(false)) {
        return;
    }

    if (hub_ && subscription_id_ != 0) {
        hub_->Unsubscribe(subscription_id_);
        subscription_id_ = 0;
    }
    hub_.reset();
}

bool RealSenseCameraDriver::IsRunning() const { return running_.load(); }

void RealSenseCameraDriver::SetSampleCallback(SampleCallback callback) {
    callback_ = std::move(callback);
}

std::shared_ptr<SensorDriver> CreateRealSenseCameraDriver(
    const SensorId& id, const DriverParams& params) {
    return std::make_shared<RealSenseCameraDriver>(id, params);
}

}  // namespace hardware
}  // namespace autodriver

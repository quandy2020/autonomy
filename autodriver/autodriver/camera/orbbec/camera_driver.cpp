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

#include "autodriver/camera/orbbec/camera_driver.hpp"

#include <utility>

#include "autodriver/types/sensor_sample.hpp"
#include "autolink/common/log.hpp"
#include "autolink/time/time.hpp"

namespace autodriver {
namespace hardware {
namespace {

autolink::Time TimeFromMilliseconds(const double ms) {
    return autolink::Time(static_cast<std::uint64_t>(ms * 1000000.0));
}

std::string ResolveFrameId(const DriverParams& params,
                           const orbbec::StreamKind stream) {
    const std::string configured = GetString(params, "frame_id");
    if (!configured.empty()) {
        return configured;
    }
    return orbbec::DefaultFrameId(stream);
}

}  // namespace

OrbbecCameraDriver::OrbbecCameraDriver(SensorId id, DriverParams params)
    : id_(std::move(id)),
      params_(std::move(params)),
      stream_(orbbec::ParseStreamKind(GetString(params_, "stream"),
                                      orbbec::StreamKind::kColor)),
      width_(ParseInt(params_, "width", 640)),
      height_(ParseInt(params_, "height", 480)),
      fps_(ParseInt(params_, "fps", 30)) {}

OrbbecCameraDriver::~OrbbecCameraDriver() { Stop(); }

bool OrbbecCameraDriver::Start() {
    if (running_.exchange(true)) {
        return true;
    }
    if (!io::OrbbecAvailable()) {
        AERROR << "Orbbec camera backend requires OrbbecSDK (id=" << id_
               << ")";
        running_ = false;
        return false;
    }

    hub_ = io::OrbbecDeviceHub::Acquire(params_);
    const SensorId id = id_;
    const std::string frame_id = ResolveFrameId(params_, stream_);
    SampleCallback callback = callback_;
    subscription_id_ = hub_->SubscribeVideo(
        stream_, width_, height_, fps_,
        [id, frame_id, callback](io::OrbbecVideoFrame frame) {
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
        AERROR << "Orbbec camera start failed: " << hub_->last_error();
        hub_->Unsubscribe(subscription_id_);
        subscription_id_ = 0;
        running_ = false;
        return false;
    }
    return true;
}

void OrbbecCameraDriver::Stop() {
    if (!running_.exchange(false)) {
        return;
    }
    if (hub_ && subscription_id_ != 0) {
        hub_->Unsubscribe(subscription_id_);
        subscription_id_ = 0;
    }
    hub_.reset();
}

bool OrbbecCameraDriver::IsRunning() const { return running_.load(); }

void OrbbecCameraDriver::SetSampleCallback(SampleCallback callback) {
    callback_ = std::move(callback);
}

std::shared_ptr<SensorDriver> CreateOrbbecCameraDriver(
    const SensorId& id, const DriverParams& params) {
    if (!io::OrbbecAvailable()) {
        AERROR << "Orbbec camera backend not available (id=" << id
               << "); install OrbbecSDK and rebuild with "
                  "AUTODRIVER_WITH_ORBBEC=ON";
        return nullptr;
    }
    return std::make_shared<OrbbecCameraDriver>(id, params);
}

}  // namespace hardware
}  // namespace autodriver

#include "autodriver/camera/backend_register.hpp"

REGISTER_CAMERA_BACKEND(orbbec_camera, "orbbec",
                        autodriver::hardware::CreateOrbbecCameraDriver);

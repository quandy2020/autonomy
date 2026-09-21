/*
 * Copyright 2026 The Openbot Authors
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

#include "autonomy/localization/atla2/sensor/camera/camera_sensor.hpp"

namespace autonomy::localization::atla2 {

CameraSensor::CameraSensor(std::string name) : name_(std::move(name)) {}

bool CameraSensor::Init(const Atla2Config& cfg) {
  K_.fx = cfg.cam_fx;
  K_.fy = cfg.cam_fy;
  K_.cx = cfg.cam_cx;
  K_.cy = cfg.cam_cy;
  return true;
}

bool CameraSensor::Start() {
  running_ = true;
  return true;
}

void CameraSensor::Stop() { running_ = false; }

TimeStamp CameraSensor::LatestTime() const {
  std::lock_guard<std::mutex> lock(mu_);
  return latest_;
}

void CameraSensor::Push(const ImageFrame& frame) {
  std::lock_guard<std::mutex> lock(mu_);
  q_.push_back(frame);
  latest_ = frame.t;
  while (q_.size() > 8) {
    q_.pop_front();
  }
}

bool CameraSensor::Pop(ImageFrame* out) {
  if (!out) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mu_);
  if (q_.empty()) {
    return false;
  }
  *out = std::move(q_.front());
  q_.pop_front();
  return true;
}

ImageFrame CameraSensor::Undistort(const ImageFrame& in, const CameraIntrinsics& K) {
  (void)K;
  // Distortion model hooks in later; pass-through for now.
  return in;
}

}  // namespace autonomy::localization::atla2

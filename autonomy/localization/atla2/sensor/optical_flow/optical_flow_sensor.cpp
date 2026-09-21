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

#include "autonomy/localization/atla2/sensor/optical_flow/optical_flow_sensor.hpp"

namespace autonomy::localization::atla2 {

OpticalFlowSensor::OpticalFlowSensor(std::string name) : name_(std::move(name)) {}

bool OpticalFlowSensor::Init(const Atla2Config& cfg) {
  fx_ = cfg.cam_fx;
  fy_ = cfg.cam_fy;
  return true;
}

bool OpticalFlowSensor::Start() {
  running_ = true;
  return true;
}

void OpticalFlowSensor::Stop() { running_ = false; }

TimeStamp OpticalFlowSensor::LatestTime() const {
  std::lock_guard<std::mutex> lock(mu_);
  return latest_;
}

void OpticalFlowSensor::Push(const OpticalFlowSample& s, double height_m) {
  OpticalFlowSample c = s;
  if (height_m > 0.05 && fx_ > 1.0 && fy_ > 1.0) {
    // v = flow_px * height / f   (per-frame; caller may scale by fps)
    c.velocity_xy = Vec2(c.flow_px.x() * height_m / fx_, c.flow_px.y() * height_m / fy_);
  }
  if (c.quality < 0.0) {
    c.quality = 0.0;
  }
  if (c.quality > 1.0) {
    c.quality = 1.0;
  }
  c.valid = c.quality > 0.1;
  std::lock_guard<std::mutex> lock(mu_);
  q_.push_back(c);
  latest_ = c.t;
  while (q_.size() > 100) {
    q_.pop_front();
  }
}

bool OpticalFlowSensor::Pop(OpticalFlowSample* out) {
  if (!out) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mu_);
  if (q_.empty()) {
    return false;
  }
  *out = q_.front();
  q_.pop_front();
  return true;
}

}  // namespace autonomy::localization::atla2

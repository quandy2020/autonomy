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

#include "autonomy/localization/atla2/frontend/vio/vio_frontend.hpp"

#include "autonomy/localization/atla2/frontend/imu_preintegration.hpp"

namespace autonomy::localization::atla2 {

bool VioFrontend::Init(const Atla2Config& cfg) {
  cfg_ = cfg;
  Reset();
  return true;
}

void VioFrontend::Reset() {
  result_ = OdometryResult{};
  initialized_ = false;
  frame_count_ = 0;
}

bool VioFrontend::Process(const SensorData& data) {
  if (!data.has_imu && !data.has_image) {
    return false;
  }
  result_.t = data.t;
  if (!initialized_) {
    // Stationary init: assume gravity along -Z in body when nearly static.
    if (data.has_imu && !data.imu.empty()) {
      Vec3 acc = Vec3::Zero();
      for (const auto& s : data.imu) {
        acc += s.accel;
      }
      acc /= static_cast<Scalar>(data.imu.size());
      const Scalar n = acc.norm();
      if (n > 1.0) {
        gravity_ = -acc.normalized() * 9.81;
      }
    }
    result_.pose = Se3Identity();
    result_.velocity = Vec3::Zero();
    result_.valid = true;
    initialized_ = true;
    ++frame_count_;
    return true;
  }

  if (data.has_imu && data.imu.size() >= 2) {
    ImuPropagate(data.imu, result_.accel_bias, result_.gyro_bias, gravity_,
                 &result_.pose, &result_.velocity);
  }
  // Vision update placeholder: keep IMU pose; landmarks filled when tracker lands.
  if (data.has_image) {
    ++frame_count_;
  }
  result_.valid = true;
  result_.cov.matrix = Mat66::Identity() * 0.01;
  return true;
}

bool VioFrontend::GetResult(OdometryResult* out) {
  if (!out || !result_.valid) {
    return false;
  }
  *out = result_;
  return true;
}

}  // namespace autonomy::localization::atla2

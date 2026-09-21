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

#include "autonomy/localization/atla2/frontend/vio/initialization.hpp"

namespace autonomy::localization::atla2 {

void VioInitialization::Reset() {
  samples_ = 0;
  acc_sum_.setZero();
  gyr_sum_.setZero();
  acc_var_ = 0.0;
}

bool VioInitialization::FeedImu(const ImuSample& s) {
  ++samples_;
  acc_sum_ += s.accel;
  gyr_sum_ += s.gyro;
  return Ready();
}

bool VioInitialization::TryFinalize(VioInitResult* out) {
  if (!out || !Ready()) {
    return false;
  }
  const Vec3 mean_a = acc_sum_ / static_cast<Scalar>(samples_);
  const Vec3 mean_g = gyr_sum_ / static_cast<Scalar>(samples_);
  const Scalar n = mean_a.norm();
  if (n < 1.0) {
    return false;
  }
  out->ok = true;
  out->gravity = -mean_a.normalized() * 9.81;
  out->accel_bias = mean_a + out->gravity;  // residual as bias approx
  out->gyro_bias = mean_g;
  out->T_world_imu = Se3Identity();
  return true;
}

}  // namespace autonomy::localization::atla2

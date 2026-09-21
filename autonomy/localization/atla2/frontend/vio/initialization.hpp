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

#pragma once

#include "autonomy/localization/atla2/common/types.hpp"
#include "autonomy/localization/atla2/sensor/interface/types.hpp"

namespace autonomy::localization::atla2 {

struct VioInitResult {
  bool ok = false;
  Vec3 gravity = Vec3(0, 0, -9.81);
  Vec3 accel_bias = Vec3::Zero();
  Vec3 gyro_bias = Vec3::Zero();
  SE3 T_world_imu = Se3Identity();
};

//! Stationary visual-inertial initialization (gravity + bias).
class VioInitialization {
 public:
  //! Accumulate IMU until enough static samples, then estimate gravity/bias.
  bool FeedImu(const ImuSample& s);
  void Reset();
  bool TryFinalize(VioInitResult* out);
  bool Ready() const { return samples_ >= min_samples_; }

 private:
  int samples_ = 0;
  int min_samples_ = 50;
  Vec3 acc_sum_ = Vec3::Zero();
  Vec3 gyr_sum_ = Vec3::Zero();
  Scalar acc_var_ = 0.0;
};

}  // namespace autonomy::localization::atla2

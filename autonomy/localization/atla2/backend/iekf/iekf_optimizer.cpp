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

#include "autonomy/localization/atla2/backend/iekf/iekf_optimizer.hpp"

namespace autonomy::localization::atla2 {

bool IekfOptimizer::Init(const Atla2Config& cfg) {
  cfg_ = cfg;
  Reset();
  return true;
}

void IekfOptimizer::Reset() {
  state_ = OdometryResult{};
  has_state_ = false;
}

bool IekfOptimizer::Update(const OdometryResult& odom) {
  if (!odom.valid) {
    return false;
  }
  if (!has_state_) {
    state_ = odom;
    has_state_ = true;
    return true;
  }

  // Simple IEKF-like blend: trust newer odom with covariance weighting.
  const Mat66& P = state_.cov.matrix;
  const Mat66& R = odom.cov.matrix;
  const Mat66 S = P + R;
  const Mat66 K = P * S.inverse();

  const Vec3 dp = Se3Translation(odom.pose) - Se3Translation(state_.pose);
  const Vec3 p = Se3Translation(state_.pose) + K.block<3, 3>(0, 0) * dp;
  const Vec3 v = state_.velocity + K.block<3, 3>(3, 3) * (odom.velocity - state_.velocity);

  // SLERP rotation toward measurement.
  Quat q = Se3Rotation(state_.pose).slerp(0.5, Se3Rotation(odom.pose));
  state_.pose = MakeSe3(p, q.normalized());
  state_.velocity = v;
  state_.gyro_bias = 0.95 * state_.gyro_bias + 0.05 * odom.gyro_bias;
  state_.accel_bias = 0.95 * state_.accel_bias + 0.05 * odom.accel_bias;
  state_.cov.matrix = (Mat66::Identity() - K) * P;
  state_.t = odom.t;
  state_.landmarks = odom.landmarks;
  state_.local_map = odom.local_map;
  state_.valid = true;
  return true;
}

bool IekfOptimizer::GetState(OdometryResult* out) const {
  if (!out || !has_state_) {
    return false;
  }
  *out = state_;
  return true;
}

}  // namespace autonomy::localization::atla2

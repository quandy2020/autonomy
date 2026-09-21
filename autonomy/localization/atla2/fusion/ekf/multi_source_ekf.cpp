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

#include "autonomy/localization/atla2/fusion/ekf/multi_source_ekf.hpp"

#include <algorithm>

namespace autonomy::localization::atla2 {
namespace {

OdometryResult BlendPose(const OdometryResult& prior, const OdometryResult& obs,
                         double w) {
  OdometryResult out = prior;
  const Vec3 p =
      (1.0 - w) * Se3Translation(prior.pose) + w * Se3Translation(obs.pose);
  const Quat q =
      Se3Rotation(prior.pose).slerp(w, Se3Rotation(obs.pose)).normalized();
  out.pose = MakeSe3(p, q);
  out.velocity = (1.0 - w) * prior.velocity + w * obs.velocity;
  out.cov.matrix =
      (1.0 - w) * prior.cov.matrix + w * obs.cov.matrix + 1e-6 * Mat66::Identity();
  out.t = obs.t;
  out.landmarks = obs.landmarks.empty() ? prior.landmarks : obs.landmarks;
  out.local_map = obs.local_map.empty() ? prior.local_map : obs.local_map;
  out.valid = true;
  return out;
}

}  // namespace

bool MultiSourceEkf::Init(const Atla2Config& cfg) {
  cfg_ = cfg;
  gate_.Init(cfg_);
  Reset();
  return true;
}

void MultiSourceEkf::Reset() {
  state_ = OdometryResult{};
  has_state_ = false;
  gate_.Reset();
}

bool MultiSourceEkf::Predict(const OdometryResult& prior) {
  if (!prior.valid) {
    return false;
  }
  if (!has_state_) {
    state_ = prior;
    has_state_ = true;
    return true;
  }
  // Process noise inflation between updates.
  state_.pose = prior.pose;
  state_.velocity = prior.velocity;
  state_.t = prior.t;
  state_.cov.matrix += 1e-3 * Mat66::Identity();
  state_.gyro_bias = prior.gyro_bias;
  state_.accel_bias = prior.accel_bias;
  state_.landmarks = prior.landmarks;
  state_.local_map = prior.local_map;
  state_.valid = true;
  return true;
}

bool MultiSourceEkf::UpdatePose(const OdometryResult& obs) {
  if (!obs.valid) {
    return false;
  }
  if (!has_state_) {
    state_ = obs;
    has_state_ = true;
    return true;
  }
  const double w = gate_.Weight(state_, obs, ObservationKind::kPose);
  if (w <= 0.0) {
    return false;
  }
  // Covariance-weighted Kalman-like gain on translation block.
  const Mat33 P = state_.cov.matrix.block<3, 3>(0, 0);
  const Mat33 R = obs.cov.matrix.block<3, 3>(0, 0) + 1e-6 * Mat33::Identity();
  const Mat33 S = P + R;
  const Mat33 K = P * S.inverse();
  const double k_scalar =
      std::min(1.0, std::max(0.0, K.trace() / 3.0)) * w;

  state_ = BlendPose(state_, obs, k_scalar);
  state_.cov.matrix.block<3, 3>(0, 0) = (Mat33::Identity() - K) * P;
  return true;
}

bool MultiSourceEkf::UpdatePosition(const Vec3& p, const Mat33& R, TimeStamp t) {
  if (!has_state_) {
    return false;
  }
  Mat33 S = state_.cov.matrix.block<3, 3>(0, 0) + R + 1e-6 * Mat33::Identity();
  const Mat33 K = state_.cov.matrix.block<3, 3>(0, 0) * S.inverse();
  const Vec3 dp = p - Se3Translation(state_.pose);
  const Vec3 p_new = Se3Translation(state_.pose) + K * dp;
  state_.pose = MakeSe3(p_new, Se3Rotation(state_.pose));
  state_.cov.matrix.block<3, 3>(0, 0) =
      (Mat33::Identity() - K) * state_.cov.matrix.block<3, 3>(0, 0);
  state_.t = t;
  return true;
}

bool MultiSourceEkf::UpdateAltitude(double z, double sigma, TimeStamp t) {
  if (!has_state_ || sigma <= 0.0) {
    return false;
  }
  const double pzz = state_.cov.matrix(2, 2);
  const double S = pzz + sigma * sigma;
  const double K = pzz / S;
  Vec3 p = Se3Translation(state_.pose);
  p.z() = p.z() + K * (z - p.z());
  state_.pose = MakeSe3(p, Se3Rotation(state_.pose));
  state_.cov.matrix(2, 2) = (1.0 - K) * pzz;
  state_.t = t;
  return true;
}

bool MultiSourceEkf::GetState(OdometryResult* out) const {
  if (!out || !has_state_) {
    return false;
  }
  *out = state_;
  return true;
}

}  // namespace autonomy::localization::atla2

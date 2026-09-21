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

#include "autonomy/localization/atla2/fusion/factor_graph/fusion_graph.hpp"

#include <algorithm>

namespace autonomy::localization::atla2 {

bool FusionGraph::Init(const Atla2Config& cfg) {
  cfg_ = cfg;
  gate_.Init(cfg_);
  max_factors_ = std::max(8, cfg_.window_size * 3);
  Reset();
  return true;
}

void FusionGraph::Reset() {
  factors_.clear();
  state_ = OdometryResult{};
  has_state_ = false;
  gate_.Reset();
}

void FusionGraph::Trim() {
  while (static_cast<int>(factors_.size()) > max_factors_) {
    factors_.erase(factors_.begin());
  }
}

void FusionGraph::AddOdometry(const OdometryResult& odom) {
  if (!odom.valid) {
    return;
  }
  if (has_state_ && !gate_.Accept(state_, odom)) {
    return;
  }
  FusionFactor f;
  f.type = FusionFactor::Type::kOdomRelative;
  f.pose = odom;
  f.weight = 1.0;
  f.t = odom.t;
  factors_.push_back(f);
  Trim();
}

void FusionGraph::AddAbsolutePosition(const Vec3& p, double weight, TimeStamp t) {
  FusionFactor f;
  f.type = FusionFactor::Type::kAbsolutePosition;
  f.position = p;
  f.weight = weight;
  f.t = t;
  factors_.push_back(f);
  Trim();
}

void FusionGraph::AddAltitude(double z, double weight, TimeStamp t) {
  FusionFactor f;
  f.type = FusionFactor::Type::kAltitude;
  f.altitude = z;
  f.weight = weight;
  f.t = t;
  factors_.push_back(f);
  Trim();
}

bool FusionGraph::Optimize(OdometryResult* out) {
  if (!out || factors_.empty()) {
    return false;
  }

  Vec3 p_acc = Vec3::Zero();
  double w_acc = 0.0;
  Quat q = Quat::Identity();
  bool have_pose = false;
  TimeStamp t = kInvalidTime;
  OdometryResult last_odom;

  for (const auto& f : factors_) {
    if (f.type == FusionFactor::Type::kOdomRelative ||
        f.type == FusionFactor::Type::kPriorPose) {
      if (!f.pose.valid) {
        continue;
      }
      const double w = std::max(1e-6, f.weight);
      p_acc += w * Se3Translation(f.pose.pose);
      w_acc += w;
      if (!have_pose) {
        q = Se3Rotation(f.pose.pose);
        have_pose = true;
        last_odom = f.pose;
      } else {
        q = q.slerp(w / w_acc, Se3Rotation(f.pose.pose)).normalized();
      }
      t = f.t;
    } else if (f.type == FusionFactor::Type::kAbsolutePosition) {
      const double w = std::max(1e-6, f.weight);
      p_acc += w * f.position;
      w_acc += w;
      t = f.t;
    } else if (f.type == FusionFactor::Type::kAltitude && has_state_) {
      Vec3 p = Se3Translation(state_.pose);
      const double w = std::max(1e-6, f.weight);
      p.z() = (p.z() + w * f.altitude) / (1.0 + w);
      state_.pose = MakeSe3(p, Se3Rotation(state_.pose));
      t = f.t;
    }
  }

  if (w_acc <= 0.0 && !has_state_) {
    return false;
  }

  if (w_acc > 0.0) {
    const Vec3 p = p_acc / w_acc;
    if (!have_pose && has_state_) {
      q = Se3Rotation(state_.pose);
    }
    state_.pose = MakeSe3(p, q.normalized());
    if (last_odom.valid) {
      state_.velocity = last_odom.velocity;
      state_.landmarks = last_odom.landmarks;
      state_.local_map = last_odom.local_map;
      state_.cov = last_odom.cov;
    }
    state_.t = t;
    state_.valid = true;
    has_state_ = true;
  }

  *out = state_;
  return has_state_;
}

bool FusionGraph::GetState(OdometryResult* out) const {
  if (!out || !has_state_) {
    return false;
  }
  *out = state_;
  return true;
}

}  // namespace autonomy::localization::atla2

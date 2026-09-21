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

#include <vector>

#include "autonomy/localization/atla2/common/config.hpp"
#include "autonomy/localization/atla2/frontend/frontend_base.hpp"
#include "autonomy/localization/atla2/fusion/outlier_rejection/outlier_rejection.hpp"

namespace autonomy::localization::atla2 {

struct FusionFactor {
  enum class Type { kPriorPose, kOdomRelative, kAbsolutePosition, kAltitude };
  Type type = Type::kPriorPose;
  OdometryResult pose;
  Vec3 position = Vec3::Zero();
  double altitude = 0.0;
  double weight = 1.0;
  TimeStamp t = kInvalidTime;
};

//! Lightweight factor-graph style multi-source fusion (sliding batch LS).
//! Used when FusionStyle::kTight — denser coupling than MultiSourceEkf.
class FusionGraph {
 public:
  bool Init(const Atla2Config& cfg);
  void Reset();

  void AddOdometry(const OdometryResult& odom);
  void AddAbsolutePosition(const Vec3& p, double weight, TimeStamp t);
  void AddAltitude(double z, double weight, TimeStamp t);

  //! Solve current factor batch → fused state.
  bool Optimize(OdometryResult* out);

  bool GetState(OdometryResult* out) const;
  int factor_count() const { return static_cast<int>(factors_.size()); }

 private:
  void Trim();

  Atla2Config cfg_;
  OutlierRejection gate_;
  std::vector<FusionFactor> factors_;
  OdometryResult state_;
  bool has_state_ = false;
  int max_factors_ = 32;
};

}  // namespace autonomy::localization::atla2

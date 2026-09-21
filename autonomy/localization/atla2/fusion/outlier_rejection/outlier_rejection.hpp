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

#include "autonomy/localization/atla2/common/config.hpp"
#include "autonomy/localization/atla2/frontend/frontend_base.hpp"

namespace autonomy::localization::atla2 {

enum class ObservationKind {
  kPose = 0,
  kVelocity,
  kPosition,
};

struct RejectStats {
  int accepted = 0;
  int rejected = 0;
  double last_mahalanobis = 0.0;
};

//! Chi-square / Mahalanobis gating for multi-source pose observations.
class OutlierRejection {
 public:
  bool Init(const Atla2Config& cfg);
  void Reset();

  //! Returns true if observation passes the gate against `prior`.
  bool Accept(const OdometryResult& prior, const OdometryResult& obs,
              ObservationKind kind = ObservationKind::kPose);

  //! Soft weight in [0,1]: 1 = fully trust obs, 0 = reject.
  double Weight(const OdometryResult& prior, const OdometryResult& obs,
                ObservationKind kind = ObservationKind::kPose) const;

  const RejectStats& stats() const { return stats_; }
  void set_chi2_threshold(double t) { chi2_threshold_ = t; }

 private:
  double MahalanobisPose(const OdometryResult& prior,
                         const OdometryResult& obs) const;

  Atla2Config cfg_;
  double chi2_threshold_ = 16.0;  // ~dof=6, p≈0.01
  mutable RejectStats stats_;
};

}  // namespace autonomy::localization::atla2

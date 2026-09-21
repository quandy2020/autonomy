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

#include "autonomy/localization/atla2/fusion/outlier_rejection/outlier_rejection.hpp"

namespace autonomy::localization::atla2 {

bool OutlierRejection::Init(const Atla2Config& cfg) {
  cfg_ = cfg;
  Reset();
  return true;
}

void OutlierRejection::Reset() { stats_ = RejectStats{}; }

double OutlierRejection::MahalanobisPose(const OdometryResult& prior,
                                         const OdometryResult& obs) const {
  const Vec3 dp = Se3Translation(obs.pose) - Se3Translation(prior.pose);
  Mat33 S = prior.cov.matrix.block<3, 3>(0, 0) + obs.cov.matrix.block<3, 3>(0, 0);
  // Regularize to avoid singular S on cold start.
  S += 1e-6 * Mat33::Identity();
  return dp.transpose() * S.inverse() * dp;
}

bool OutlierRejection::Accept(const OdometryResult& prior,
                              const OdometryResult& obs, ObservationKind kind) {
  (void)kind;
  if (!prior.valid || !obs.valid) {
    ++stats_.rejected;
    return false;
  }
  const double d2 = MahalanobisPose(prior, obs);
  stats_.last_mahalanobis = d2;
  if (d2 > chi2_threshold_) {
    ++stats_.rejected;
    return false;
  }
  ++stats_.accepted;
  return true;
}

double OutlierRejection::Weight(const OdometryResult& prior,
                                const OdometryResult& obs,
                                ObservationKind kind) const {
  (void)kind;
  if (!prior.valid || !obs.valid) {
    return 0.0;
  }
  const double d2 = MahalanobisPose(prior, obs);
  stats_.last_mahalanobis = d2;
  if (d2 >= chi2_threshold_) {
    return 0.0;
  }
  // Linear taper toward threshold.
  return 1.0 - (d2 / chi2_threshold_);
}

}  // namespace autonomy::localization::atla2

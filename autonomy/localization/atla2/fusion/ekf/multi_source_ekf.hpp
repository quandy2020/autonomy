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
#include "autonomy/localization/atla2/fusion/outlier_rejection/outlier_rejection.hpp"
#include "autonomy/localization/atla2/sensor/types.hpp"

namespace autonomy::localization::atla2 {

//! Loose multi-source EKF: predict with IMU-coast / prior odom, update with
//! gated VIO / LIO / GPS / baro position observations.
class MultiSourceEkf {
 public:
  bool Init(const Atla2Config& cfg);
  void Reset();

  //! Propagate with a motion prior (frontend odom or IMU coast).
  bool Predict(const OdometryResult& prior);

  //! Position / pose measurement update (VIO or LIO).
  bool UpdatePose(const OdometryResult& obs);

  //! Absolute position (GPS ENU / local frame).
  bool UpdatePosition(const Vec3& p, const Mat33& R, TimeStamp t);

  //! Scalar altitude (barometer → z).
  bool UpdateAltitude(double z, double sigma, TimeStamp t);

  bool GetState(OdometryResult* out) const;
  bool has_state() const { return has_state_; }

 private:
  Atla2Config cfg_;
  OutlierRejection gate_;
  OdometryResult state_;
  bool has_state_ = false;
};

}  // namespace autonomy::localization::atla2

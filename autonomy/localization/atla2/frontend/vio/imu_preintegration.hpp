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

#include "autonomy/localization/atla2/sensor/types.hpp"

namespace autonomy::localization::atla2 {

//! Mid-point IMU preintegration between two frames (manifold SO3 + p/v).
class ImuPreintegration {
 public:
  void Reset(TimeStamp t0);
  void Push(const ImuSample& s, const Vec3& ba, const Vec3& bg);
  void Finish(TimeStamp t1);

  TimeStamp t0() const { return t0_; }
  TimeStamp t1() const { return t1_; }
  double Dt() const { return dt_; }

  const Mat33& DeltaR() const { return dR_; }
  const Vec3& DeltaV() const { return dV_; }
  const Vec3& DeltaP() const { return dP_; }

 private:
  TimeStamp t0_ = kInvalidTime;
  TimeStamp t1_ = kInvalidTime;
  double dt_ = 0.0;
  Mat33 dR_ = Mat33::Identity();
  Vec3 dV_ = Vec3::Zero();
  Vec3 dP_ = Vec3::Zero();
  ImuSample last_;
  bool has_last_ = false;
};

//! Propagate SE3 pose + velocity with IMU mid-point (gravity in world).
void ImuPropagate(const std::vector<ImuSample>& imu, const Vec3& ba, const Vec3& bg,
                  const Vec3& gravity, SE3* pose, Vec3* velocity);

}  // namespace autonomy::localization::atla2

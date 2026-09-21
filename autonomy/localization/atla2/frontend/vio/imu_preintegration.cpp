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

#include "autonomy/localization/atla2/frontend/imu_preintegration.hpp"

#include "autonomy/localization/atla2/common/time.hpp"

namespace autonomy::localization::atla2 {

namespace {

Mat33 Skew(const Vec3& v) {
  Mat33 m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}

Mat33 ExpSo3(const Vec3& w) {
  const Scalar angle = w.norm();
  if (angle < Scalar(1e-10)) {
    return Mat33::Identity() + Skew(w);
  }
  const Vec3 axis = w / angle;
  return Eigen::AngleAxis<Scalar>(angle, axis).toRotationMatrix();
}

}  // namespace

void ImuPreintegration::Reset(TimeStamp t0) {
  t0_ = t0;
  t1_ = t0;
  dt_ = 0.0;
  dR_ = Mat33::Identity();
  dV_ = Vec3::Zero();
  dP_ = Vec3::Zero();
  has_last_ = false;
}

void ImuPreintegration::Push(const ImuSample& s, const Vec3& ba, const Vec3& bg) {
  if (!has_last_) {
    last_ = s;
    has_last_ = true;
    t0_ = s.t;
    t1_ = s.t;
    return;
  }
  const double dt = StampToSec(s.t - last_.t);
  if (dt <= 0.0 || dt > 0.05) {
    last_ = s;
    t1_ = s.t;
    return;
  }
  const Vec3 w = 0.5 * (last_.gyro + s.gyro) - bg;
  const Vec3 a = 0.5 * (last_.accel + s.accel) - ba;
  const Mat33 dR = ExpSo3(w * dt);
  dP_ = dP_ + dV_ * dt + 0.5 * dR_ * a * dt * dt;
  dV_ = dV_ + dR_ * a * dt;
  dR_ = dR_ * dR;
  dt_ += dt;
  last_ = s;
  t1_ = s.t;
}

void ImuPreintegration::Finish(TimeStamp t1) { t1_ = t1; }

void ImuPropagate(const std::vector<ImuSample>& imu, const Vec3& ba, const Vec3& bg,
                  const Vec3& gravity, SE3* pose, Vec3* velocity) {
  if (!pose || !velocity || imu.size() < 2) {
    return;
  }
  Vec3 p = Se3Translation(*pose);
  Mat33 R = Se3RotationMatrix(*pose);
  Vec3 v = *velocity;

  for (size_t i = 1; i < imu.size(); ++i) {
    const double dt = StampToSec(imu[i].t - imu[i - 1].t);
    if (dt <= 0.0 || dt > 0.05) {
      continue;
    }
    const Vec3 w = 0.5 * (imu[i - 1].gyro + imu[i].gyro) - bg;
    const Vec3 a_body = 0.5 * (imu[i - 1].accel + imu[i].accel) - ba;
    const Mat33 dR = ExpSo3(w * dt);
    const Vec3 a_world = R * a_body + gravity;
    p = p + v * dt + 0.5 * a_world * dt * dt;
    v = v + a_world * dt;
    R = R * dR;
  }
  *pose = MakeSe3(p, Quat(R));
  *velocity = v;
}

}  // namespace autonomy::localization::atla2

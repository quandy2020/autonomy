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

#include "Eigen/Geometry"

#include "autonomy/localization/atla2/common/types.hpp"

namespace autonomy::localization::atla2 {

inline void Se3ToPq(const SE3& T, double* p, double* aa) {
  const Vec3 t = Se3Translation(T);
  p[0] = t.x();
  p[1] = t.y();
  p[2] = t.z();
  const Eigen::AngleAxisd A(Se3RotationMatrix(T));
  const Vec3 v = A.angle() * A.axis();
  aa[0] = v.x();
  aa[1] = v.y();
  aa[2] = v.z();
}

inline SE3 PqToSe3(const double* p, const double* aa) {
  const Vec3 w(aa[0], aa[1], aa[2]);
  const Scalar n = w.norm();
  Mat33 R = Mat33::Identity();
  if (n > Scalar(1e-12)) {
    R = Eigen::AngleAxisd(n, w / n).toRotationMatrix();
  }
  return MakeSe3(Vec3(p[0], p[1], p[2]), Quat(R));
}

inline void Vec3ToArray(const Vec3& v, double* a) {
  a[0] = v.x();
  a[1] = v.y();
  a[2] = v.z();
}

inline Vec3 ArrayToVec3(const double* a) { return Vec3(a[0], a[1], a[2]); }

struct CameraModel {
  double fx = 320.0;
  double fy = 320.0;
  double cx = 320.0;
  double cy = 240.0;
};

}  // namespace autonomy::localization::atla2

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

#include "ceres/ceres.h"
#include "ceres/rotation.h"

namespace autonomy::localization::atla2 {

//! Monocular reprojection (camera frame ≡ body).
struct ReprojectionFactor {
  ReprojectionFactor(double u, double v, double fx, double fy, double cx, double cy,
                     double weight)
      : u_(u), v_(v), fx_(fx), fy_(fy), cx_(cx), cy_(cy), weight_(weight) {}

  template <typename T>
  bool operator()(const T* const p, const T* const aa, const T* const landmark,
                  T* residual) const {
    const T dp[3] = {landmark[0] - p[0], landmark[1] - p[1], landmark[2] - p[2]};
    T aa_inv[3] = {-aa[0], -aa[1], -aa[2]};
    T Xb[3];
    ceres::AngleAxisRotatePoint(aa_inv, dp, Xb);

    const T eps = T(1e-6);
    T z = Xb[2];
    if (z >= T(0) && z < eps) {
      z = eps;
    } else if (z < T(0) && z > -eps) {
      z = -eps;
    }
    const T invz = T(1.0) / z;
    residual[0] = T(weight_) * (T(fx_) * Xb[0] * invz + T(cx_) - T(u_));
    residual[1] = T(weight_) * (T(fy_) * Xb[1] * invz + T(cy_) - T(v_));
    return true;
  }

  static ceres::CostFunction* Create(double u, double v, double fx, double fy, double cx,
                                     double cy, double w) {
    return new ceres::AutoDiffCostFunction<ReprojectionFactor, 2, 3, 3, 3>(
        new ReprojectionFactor(u, v, fx, fy, cx, cy, w));
  }

  double u_, v_, fx_, fy_, cx_, cy_, weight_;
};

}  // namespace autonomy::localization::atla2

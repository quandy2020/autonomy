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

//! 6-DoF relative pose: measured Δp / Δaa in frame i.
struct RelativePoseFactor {
  RelativePoseFactor(const double* dp, const double* daa, double weight)
      : weight_(weight) {
    for (int i = 0; i < 3; ++i) {
      dp_[i] = dp[i];
      daa_[i] = daa[i];
    }
  }

  template <typename T>
  bool operator()(const T* const p_i, const T* const aa_i, const T* const p_j,
                  const T* const aa_j, T* residual) const {
    const T dp_w[3] = {p_j[0] - p_i[0], p_j[1] - p_i[1], p_j[2] - p_i[2]};
    // R_i^T * dp_w  via rotate by -aa_i
    T aa_inv[3] = {-aa_i[0], -aa_i[1], -aa_i[2]};
    T t_pred[3];
    ceres::AngleAxisRotatePoint(aa_inv, dp_w, t_pred);

    residual[0] = T(weight_) * (t_pred[0] - T(dp_[0]));
    residual[1] = T(weight_) * (t_pred[1] - T(dp_[1]));
    residual[2] = T(weight_) * (t_pred[2] - T(dp_[2]));

    // aa of R_i^T R_j
    T q_i[4], q_j[4], q_i_inv[4], q_rel[4];
    ceres::AngleAxisToQuaternion(aa_i, q_i);
    ceres::AngleAxisToQuaternion(aa_j, q_j);
    q_i_inv[0] = q_i[0];
    q_i_inv[1] = -q_i[1];
    q_i_inv[2] = -q_i[2];
    q_i_inv[3] = -q_i[3];
    ceres::QuaternionProduct(q_i_inv, q_j, q_rel);
    T aa_pred[3];
    ceres::QuaternionToAngleAxis(q_rel, aa_pred);
    residual[3] = T(weight_) * (aa_pred[0] - T(daa_[0]));
    residual[4] = T(weight_) * (aa_pred[1] - T(daa_[1]));
    residual[5] = T(weight_) * (aa_pred[2] - T(daa_[2]));
    return true;
  }

  static ceres::CostFunction* Create(const double* dp, const double* daa, double w) {
    return new ceres::AutoDiffCostFunction<RelativePoseFactor, 6, 3, 3, 3, 3>(
        new RelativePoseFactor(dp, daa, w));
  }

  double dp_[3];
  double daa_[3];
  double weight_;
};

}  // namespace autonomy::localization::atla2

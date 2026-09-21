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

//! IMU preintegration residual (biases held constant in this factor).
struct ImuFactor {
  ImuFactor(const double* dP, const double* dV, const double* dR_aa, double dt,
            const double* gravity, double weight)
      : dt_(dt), weight_(weight) {
    for (int i = 0; i < 3; ++i) {
      dP_[i] = dP[i];
      dV_[i] = dV[i];
      dR_aa_[i] = dR_aa[i];
      g_[i] = gravity[i];
    }
  }

  template <typename T>
  bool operator()(const T* const p_i, const T* const aa_i, const T* const v_i,
                  const T* const p_j, const T* const aa_j, const T* const v_j,
                  T* residual) const {
    const T dt = T(dt_);
    T aa_inv[3] = {-aa_i[0], -aa_i[1], -aa_i[2]};

    const T dv_w[3] = {v_j[0] - v_i[0] - T(g_[0]) * dt, v_j[1] - v_i[1] - T(g_[1]) * dt,
                       v_j[2] - v_i[2] - T(g_[2]) * dt};
    T dv_b[3];
    ceres::AngleAxisRotatePoint(aa_inv, dv_w, dv_b);

    const T dp_w[3] = {
        p_j[0] - p_i[0] - v_i[0] * dt - T(0.5) * T(g_[0]) * dt * dt,
        p_j[1] - p_i[1] - v_i[1] * dt - T(0.5) * T(g_[1]) * dt * dt,
        p_j[2] - p_i[2] - v_i[2] * dt - T(0.5) * T(g_[2]) * dt * dt};
    T dp_b[3];
    ceres::AngleAxisRotatePoint(aa_inv, dp_w, dp_b);

    residual[0] = T(weight_) * (dp_b[0] - T(dP_[0]));
    residual[1] = T(weight_) * (dp_b[1] - T(dP_[1]));
    residual[2] = T(weight_) * (dp_b[2] - T(dP_[2]));
    residual[3] = T(weight_) * (dv_b[0] - T(dV_[0]));
    residual[4] = T(weight_) * (dv_b[1] - T(dV_[1]));
    residual[5] = T(weight_) * (dv_b[2] - T(dV_[2]));

    T q_i[4], q_j[4], q_i_inv[4], q_meas[4], q_pred[4], q_err[4];
    ceres::AngleAxisToQuaternion(aa_i, q_i);
    ceres::AngleAxisToQuaternion(aa_j, q_j);
    const T dR_aa_T[3] = {T(dR_aa_[0]), T(dR_aa_[1]), T(dR_aa_[2])};
    ceres::AngleAxisToQuaternion(dR_aa_T, q_meas);
    q_i_inv[0] = q_i[0];
    q_i_inv[1] = -q_i[1];
    q_i_inv[2] = -q_i[2];
    q_i_inv[3] = -q_i[3];
    // q_pred = q_i^{-1} ⊗ q_j
    ceres::QuaternionProduct(q_i_inv, q_j, q_pred);
    // q_err = q_meas^{-1} ⊗ q_pred
    T q_meas_inv[4] = {q_meas[0], -q_meas[1], -q_meas[2], -q_meas[3]};
    ceres::QuaternionProduct(q_meas_inv, q_pred, q_err);
    T aa_err[3];
    ceres::QuaternionToAngleAxis(q_err, aa_err);
    residual[6] = T(weight_) * aa_err[0];
    residual[7] = T(weight_) * aa_err[1];
    residual[8] = T(weight_) * aa_err[2];
    return true;
  }

  static ceres::CostFunction* Create(const double* dP, const double* dV, const double* dR_aa,
                                     double dt, const double* g, double w) {
    return new ceres::AutoDiffCostFunction<ImuFactor, 9, 3, 3, 3, 3, 3, 3>(
        new ImuFactor(dP, dV, dR_aa, dt, g, w));
  }

  double dP_[3], dV_[3], dR_aa_[3], g_[3];
  double dt_;
  double weight_;
};

}  // namespace autonomy::localization::atla2

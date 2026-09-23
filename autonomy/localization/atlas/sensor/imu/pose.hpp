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
 *
 * Camera↔IMU pose helpers (ORB-SLAM3 Tcw ↔ Twb via Tcb/Tbc).
 */

/**
 * @file
 * @brief Camera ↔ IMU/body pose transforms and preintegration sqrt-info helpers.
 *
 * Conventions match ORB-SLAM3: `T_cw` is the inverse of world←camera (ORB Tcw),
 * `T_wb` is world←body; extrinsics via `Calib::T_camera_body` (Tcb).
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_SENSOR_IMU_POSE_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_SENSOR_IMU_POSE_HPP_

#include "Eigen/Cholesky"
#include "Eigen/Eigenvalues"

#include "autonomy/localization/atlas/common/types.hpp"
#include "autonomy/localization/atlas/sensor/imu/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace imu {

/**
 * @brief IMU/body pose from camera pose: @f$ T_{wb}=T_{cw}^{-1} T_{cb} @f$.
 * @param T_cw Camera pose (inverse of world←camera, ORB `Tcw`).
 * @param calib Calibration containing `T_camera_body`.
 * @return world←body pose `Twb`.
 */
inline SE3 CameraPoseToImuPose(const SE3& T_cw, const Calib& calib) {
    return T_cw.inverse() * calib.T_camera_body;
}

/**
 * @brief Camera pose from IMU/body pose: @f$ T_{cw}=T_{cb} T_{wb}^{-1} @f$.
 * @param T_wb world←body.
 * @param calib Calibration containing `T_camera_body`.
 * @return ORB-style `Tcw`.
 */
inline SE3 ImuPoseToCameraPose(const SE3& T_wb, const Calib& calib) {
    return calib.T_camera_body * T_wb.inverse();
}

/**
 * @brief If calib is unset, return a copy with identity extrinsics and default noise.
 * @param calib Input calibration.
 * @return `calib` as-is when `is_set`; otherwise default from `Set(I, …)`.
 */
inline Calib DefaultCalibOr(const Calib& calib) {
    if (calib.is_set) {
        return calib;
    }
    Calib out;
    out.Set(SE3Identity(), 1e-4, 1e-3, 1e-6, 1e-5);
    return out;
}

/**
 * @brief Upper-triangular sqrt information from 3×3 covariance (LLT Lᵀ).
 * @param cov Symmetric positive-definite (approx.) covariance.
 * @return @f$ \sqrt{\mathrm{Info}} @f$; identity on failure.
 */
inline Eigen::Matrix3d ComputeSqrtInformation3(const Mat33& cov) {
    Mat33 C = 0.5 * (cov + cov.transpose());
    Eigen::SelfAdjointEigenSolver<Mat33> es(C);
    Vec3 eigs = es.eigenvalues();
    for (int i = 0; i < 3; ++i) {
        eigs[i] = (eigs[i] > 1e-12) ? (1.0 / eigs[i]) : 0.0;
    }
    const Mat33 Info =
        es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
    Eigen::LLT<Mat33> llt(Info);
    if (llt.info() == Eigen::Success) {
        return llt.matrixL().transpose();
    }
    return Mat33::Identity();
}

/**
 * @brief Sqrt information for inertial residual from preintegrator cov 9×9 block.
 * @param pre Preintegrator (reads `covariance.block<9,9>(0,0)`).
 * @return 9×9 upper-triangular sqrt info; identity if LLT fails.
 *
 * Usage: weight residual as `sqrt_info * e` (ORB-SLAM3 `EdgeInertial`).
 */
inline Eigen::Matrix<double, 9, 9> ComputeSqrtInformation9(
    const Preintegrator& pre) {
    Eigen::Matrix<double, 9, 9> C = pre.covariance.block<9, 9>(0, 0);
    C = 0.5 * (C + C.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> es(C);
    Eigen::Matrix<double, 9, 1> eigs = es.eigenvalues();
    for (int i = 0; i < 9; ++i) {
        eigs[i] = (eigs[i] > 1e-12) ? (1.0 / eigs[i]) : 0.0;
    }
    const Eigen::Matrix<double, 9, 9> Info =
        es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
    Eigen::LLT<Eigen::Matrix<double, 9, 9>> llt(Info);
    if (llt.info() == Eigen::Success) {
        return llt.matrixL().transpose();  // upper: residual = sqrt_info * e
    }
    return Eigen::Matrix<double, 9, 9>::Identity();
}

}  // namespace imu
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_SENSOR_IMU_POSE_HPP_

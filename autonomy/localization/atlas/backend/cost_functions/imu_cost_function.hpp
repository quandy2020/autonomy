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

/**
 * @file imu_cost_function.hpp
 * @brief IMU preintegration and bias cost functors.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_IMU_COST_FUNCTION_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_IMU_COST_FUNCTION_HPP_

#include <cmath>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "Eigen/Core"

#include "autonomy/localization/atlas/common/types.hpp"
#include "autonomy/localization/atlas/sensor/imu/pose.hpp"
#include "autonomy/localization/atlas/sensor/imu/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace backend {


/**
 * @struct autonomy::localization::atlas::backend::ImuPreintegrationBiasCostFunctor
 * @brief 9-D IMU preintegration residual with first-order bias correction
 *        (ORB EdgeInertial).
 *
 * Bias correction:
 * \(dR(b)\approx dR\,\mathrm{Exp}(J_{Rg}\delta b_g)\), and similarly for
 * \(dV/dP\) via \(J_{Vg}/J_{Va}/J_{Pg}/J_{Pa}\).
 * \(\delta b\) is relative to the biases `bg0_`/`ba0_` used during
 * preintegration.
 *
 * Ceres dimensions: residual 9; parameter blocks 6,6,3,3,6
 * (pose_i, pose_j, vel_i, vel_j, bias_i).
 *
 * Default state convention:
 * - `pose_i/j`: 6 = `[angle_axis(3), translation(3)]` for \(T_{wb}\)
 * - `vel_i/j`: 3, world-frame velocity
 * - `bias_i`: 6 = `[bg(3), ba(3)]`
 */
struct ImuPreintegrationBiasCostFunctor {
    /**
     * @brief Construct from a preintegrator and gravity.
     * @param pre Inter-keyframe preintegration result (Jacobians and covariance).
     * @param gravity World-frame gravity vector.
     * @param sqrt_info 9×9 square-root information matrix.
     */
    ImuPreintegrationBiasCostFunctor(
        const sensor::imu::Preintegrator& pre, const Vec3& gravity,
        const Eigen::Matrix<double, 9, 9>& sqrt_info)
        : dR_(pre.dR),
          dV_(pre.dV),
          dP_(pre.dP),
          JRg_(pre.JRg),
          JVg_(pre.JVg),
          JVa_(pre.JVa),
          JPg_(pre.JPg),
          JPa_(pre.JPa),
          bg0_(pre.bias.gyroscope),
          ba0_(pre.bias.accelerometer),
          dt_(pre.delta_t),
          gravity_(gravity),
          sqrt_info_(sqrt_info) {}

    /**
     * @brief AutoDiff residual evaluation.
     * @tparam T Scalar type.
     * @param pose_i \(T_{wb}\) at keyframe i.
     * @param pose_j \(T_{wb}\) at keyframe j.
     * @param vel_i Velocity at keyframe i.
     * @param vel_j Velocity at keyframe j.
     * @param bias_i Bias `[bg, ba]` at keyframe i.
     * @param[out] residuals Length-9 residual.
     * @return Always true.
     */
    template <typename T>
    bool operator()(const T* const pose_i, const T* const pose_j,
                    const T* const vel_i, const T* const vel_j,
                    const T* const bias_i, T* residuals) const {
        Eigen::Matrix<T, 3, 3> Ri;
        Eigen::Matrix<T, 3, 3> Rj;
        ceres::AngleAxisToRotationMatrix(pose_i, Ri.data());
        ceres::AngleAxisToRotationMatrix(pose_j, Rj.data());
        const Eigen::Matrix<T, 3, 1> ti(pose_i[3], pose_i[4], pose_i[5]);
        const Eigen::Matrix<T, 3, 1> tj(pose_j[3], pose_j[4], pose_j[5]);
        const Eigen::Matrix<T, 3, 1> vi(vel_i[0], vel_i[1], vel_i[2]);
        const Eigen::Matrix<T, 3, 1> vj(vel_j[0], vel_j[1], vel_j[2]);

        const Eigen::Matrix<T, 3, 1> dbg(bias_i[0] - T(bg0_[0]),
                                        bias_i[1] - T(bg0_[1]),
                                        bias_i[2] - T(bg0_[2]));
        const Eigen::Matrix<T, 3, 1> dba(bias_i[3] - T(ba0_[0]),
                                        bias_i[4] - T(ba0_[1]),
                                        bias_i[5] - T(ba0_[2]));

        const Eigen::Matrix<T, 3, 1> aa = JRg_.cast<T>() * dbg;
        T dR_delta[9];
        ceres::AngleAxisToRotationMatrix(aa.data(), dR_delta);
        Eigen::Map<const Eigen::Matrix<T, 3, 3>> dR_delta_m(dR_delta);
        const Eigen::Matrix<T, 3, 3> dR_corr = dR_.cast<T>() * dR_delta_m;
        const Eigen::Matrix<T, 3, 1> dV_corr =
            dV_.cast<T>() + JVg_.cast<T>() * dbg + JVa_.cast<T>() * dba;
        const Eigen::Matrix<T, 3, 1> dP_corr =
            dP_.cast<T>() + JPg_.cast<T>() * dbg + JPa_.cast<T>() * dba;

        const Eigen::Matrix<T, 3, 1> g = gravity_.cast<T>();
        const T dt = T(dt_);

        const Eigen::Matrix<T, 3, 3> R_err =
            dR_corr.transpose() * Ri.transpose() * Rj;
        T R_err_aa[3];
        ceres::RotationMatrixToAngleAxis(R_err.data(), R_err_aa);

        const Eigen::Matrix<T, 3, 1> v_err =
            Ri.transpose() * (vj - vi - g * dt) - dV_corr;
        const Eigen::Matrix<T, 3, 1> p_err =
            Ri.transpose() * (tj - ti - vi * dt - T(0.5) * g * dt * dt) -
            dP_corr;

        Eigen::Matrix<T, 9, 1> e;
        e << R_err_aa[0], R_err_aa[1], R_err_aa[2], v_err[0], v_err[1],
            v_err[2], p_err[0], p_err[1], p_err[2];
        const Eigen::Matrix<T, 9, 1> we = sqrt_info_.cast<T>() * e;
        for (int k = 0; k < 9; ++k) {
            residuals[k] = we[k];
        }
        return true;
    }

    /**
     * @brief Factory: builds \(\sqrt{\Lambda}\) from the preintegrator.
     * @param pre Preintegrator.
     * @param gravity World-frame gravity.
     * @return AutoDiff residual 9; blocks 6/6/3/3/6.
     */
    static ceres::CostFunction* Create(const sensor::imu::Preintegrator& pre,
                                       const Vec3& gravity) {
        return Create(pre, gravity, 1.0);
    }

    /**
     * @brief Factory with an information scale (1 = full; boundary edge uses 1e-2).
     * @param information_scale Multiplies the 9×9 information matrix.
     */
    static ceres::CostFunction* Create(const sensor::imu::Preintegrator& pre,
                                       const Vec3& gravity,
                                       double information_scale) {
        auto sqrt_info = sensor::imu::ComputeSqrtInformation9(pre);
        sqrt_info *= std::sqrt(information_scale);
        return new ceres::AutoDiffCostFunction<ImuPreintegrationBiasCostFunctor,
                                               9, 6, 6, 3, 3, 6>(
            new ImuPreintegrationBiasCostFunctor(pre, gravity, sqrt_info));
    }

    Mat33 dR_, JRg_, JVg_, JVa_, JPg_, JPa_;  ///< Preintegrated rotation and bias Jacobians
    Vec3 dV_, dP_, bg0_, ba0_, gravity_;      ///< Δv/Δp, preintegration biases, gravity
    double dt_;                               ///< Preintegration duration
    Eigen::Matrix<double, 9, 9> sqrt_info_;   ///< \(\sqrt{\Lambda}\)
};

/**
 * @struct autonomy::localization::atlas::backend::ImuPreintegrationCostFunctor
 * @brief Fixed-bias 9-D IMU residual (legacy API; bias is baked into dR/dV/dP).
 *
 * Parameter blocks: pose_i(6), pose_j(6), vel_i(3), vel_j(3); no bias block.
 * Weighting uses scalar `weight_` (not a full information matrix).
 */
struct ImuPreintegrationCostFunctor {
    /**
     * @brief Construct from preintegrated deltas.
     * @param dR Preintegrated rotation.
     * @param dV Preintegrated velocity.
     * @param dP Preintegrated position.
     * @param dt Preintegration duration.
     * @param gravity World-frame gravity.
     * @param weight Scalar residual weight (default 1.0).
     */
    ImuPreintegrationCostFunctor(const Mat33& dR, const Vec3& dV,
                                 const Vec3& dP, double dt, const Vec3& gravity,
                                 double weight = 1.0)
        : dR_(dR),
          dV_(dV),
          dP_(dP),
          dt_(dt),
          gravity_(gravity),
          weight_(weight) {}

    /**
     * @brief AutoDiff residual evaluation.
     * @tparam T Scalar type.
     * @param pose_i \(T_{wb}\) at keyframe i.
     * @param pose_j \(T_{wb}\) at keyframe j.
     * @param vel_i Velocity at keyframe i.
     * @param vel_j Velocity at keyframe j.
     * @param[out] residuals Length-9 residual.
     * @return Always true.
     */
    template <typename T>
    bool operator()(const T* const pose_i, const T* const pose_j,
                    const T* const vel_i, const T* const vel_j,
                    T* residuals) const {
        Eigen::Matrix<T, 3, 3> Ri;
        Eigen::Matrix<T, 3, 3> Rj;
        ceres::AngleAxisToRotationMatrix(pose_i, Ri.data());
        ceres::AngleAxisToRotationMatrix(pose_j, Rj.data());
        const Eigen::Matrix<T, 3, 1> ti(pose_i[3], pose_i[4], pose_i[5]);
        const Eigen::Matrix<T, 3, 1> tj(pose_j[3], pose_j[4], pose_j[5]);
        const Eigen::Matrix<T, 3, 1> vi(vel_i[0], vel_i[1], vel_i[2]);
        const Eigen::Matrix<T, 3, 1> vj(vel_j[0], vel_j[1], vel_j[2]);

        const Eigen::Matrix<T, 3, 3> dR = dR_.cast<T>();
        const Eigen::Matrix<T, 3, 1> dV = dV_.cast<T>();
        const Eigen::Matrix<T, 3, 1> dP = dP_.cast<T>();
        const Eigen::Matrix<T, 3, 1> g = gravity_.cast<T>();
        const T dt = T(dt_);
        const T w = T(weight_);

        const Eigen::Matrix<T, 3, 3> R_err =
            dR.transpose() * Ri.transpose() * Rj;
        T R_err_aa[3];
        ceres::RotationMatrixToAngleAxis(R_err.data(), R_err_aa);

        const Eigen::Matrix<T, 3, 1> v_err =
            Ri.transpose() * (vj - vi - g * dt) - dV;
        const Eigen::Matrix<T, 3, 1> p_err =
            Ri.transpose() * (tj - ti - vi * dt - T(0.5) * g * dt * dt) - dP;

        residuals[0] = w * R_err_aa[0];
        residuals[1] = w * R_err_aa[1];
        residuals[2] = w * R_err_aa[2];
        residuals[3] = w * v_err[0];
        residuals[4] = w * v_err[1];
        residuals[5] = w * v_err[2];
        residuals[6] = w * p_err[0];
        residuals[7] = w * p_err[1];
        residuals[8] = w * p_err[2];
        return true;
    }

    /**
     * @brief Factory: residual 9; blocks 6,6,3,3.
     * @return Heap-allocated CostFunction.
     */
    static ceres::CostFunction* Create(const Mat33& dR, const Vec3& dV,
                                       const Vec3& dP, double dt,
                                       const Vec3& gravity,
                                       double weight = 1.0) {
        return new ceres::AutoDiffCostFunction<ImuPreintegrationCostFunctor, 9,
                                               6, 6, 3, 3>(
            new ImuPreintegrationCostFunctor(dR, dV, dP, dt, gravity, weight));
    }

    Mat33 dR_;
    Vec3 dV_;
    Vec3 dP_;
    double dt_;
    Vec3 gravity_;
    double weight_;  ///< Scalar weight
};

/**
 * @struct autonomy::localization::atlas::backend::ImuBiasWalkCostFunctor
 * @brief Bias random walk: \(r=\sqrt{\Lambda}(b_j-b_i)\) (covariance blocks
 *        9:12 / 12:15).
 *
 * Residual dimension: 6 (gyro 3 + accel 3). Parameter blocks: bias_i(6),
 * bias_j(6).
 */
struct ImuBiasWalkCostFunctor {
    /**
     * @brief Construct with matrix square-root information.
     * @param sqrt_info_gyro Gyro bias \(\sqrt{\Lambda}\).
     * @param sqrt_info_acc Accel bias \(\sqrt{\Lambda}\).
     */
    ImuBiasWalkCostFunctor(const Mat33& sqrt_info_gyro,
                           const Mat33& sqrt_info_acc)
        : sqrt_info_gyro_(sqrt_info_gyro), sqrt_info_acc_(sqrt_info_acc) {}

    /**
     * @brief AutoDiff residual evaluation.
     * @tparam T Scalar type.
     * @param bias_i Bias at keyframe i `[bg, ba]`.
     * @param bias_j Bias at keyframe j `[bg, ba]`.
     * @param[out] residuals Length-6 residual.
     * @return Always true.
     */
    template <typename T>
    bool operator()(const T* const bias_i, const T* const bias_j,
                    T* residuals) const {
        Eigen::Matrix<T, 3, 1> dg(bias_j[0] - bias_i[0], bias_j[1] - bias_i[1],
                                  bias_j[2] - bias_i[2]);
        Eigen::Matrix<T, 3, 1> da(bias_j[3] - bias_i[3], bias_j[4] - bias_i[4],
                                  bias_j[5] - bias_i[5]);
        const Eigen::Matrix<T, 3, 1> rg = sqrt_info_gyro_.cast<T>() * dg;
        const Eigen::Matrix<T, 3, 1> ra = sqrt_info_acc_.cast<T>() * da;
        residuals[0] = rg[0];
        residuals[1] = rg[1];
        residuals[2] = rg[2];
        residuals[3] = ra[0];
        residuals[4] = ra[1];
        residuals[5] = ra[2];
        return true;
    }

    /**
     * @brief Factory with isotropic scalar information.
     * @param sqrt_info_gyro Scalar gyro weight.
     * @param sqrt_info_acc Scalar accel weight.
     * @return Heap-allocated CostFunction.
     */
    static ceres::CostFunction* Create(double sqrt_info_gyro,
                                       double sqrt_info_acc) {
        return Create(sqrt_info_gyro * Mat33::Identity(),
                      sqrt_info_acc * Mat33::Identity());
    }

    /**
     * @brief Factory with matrix information: residual 6; blocks 6,6.
     * @param sqrt_info_gyro Gyro bias \(\sqrt{\Lambda}\).
     * @param sqrt_info_acc Accel bias \(\sqrt{\Lambda}\).
     * @return Heap-allocated CostFunction.
     */
    static ceres::CostFunction* Create(const Mat33& sqrt_info_gyro,
                                       const Mat33& sqrt_info_acc) {
        return new ceres::AutoDiffCostFunction<ImuBiasWalkCostFunctor, 6, 6, 6>(
            new ImuBiasWalkCostFunctor(sqrt_info_gyro, sqrt_info_acc));
    }

    /**
     * @brief Extract \(\sqrt{\Lambda}\) from preintegrator covariance blocks.
     * @param pre Preintegrator.
     * @return Heap-allocated CostFunction.
     */
    static ceres::CostFunction* CreateFromPreintegrator(
        const sensor::imu::Preintegrator& pre) {
        return Create(
            sensor::imu::ComputeSqrtInformation3(
                pre.covariance.template block<3, 3>(9, 9)),
            sensor::imu::ComputeSqrtInformation3(
                pre.covariance.template block<3, 3>(12, 12)));
    }

    Mat33 sqrt_info_gyro_;  ///< Gyro bias \(\sqrt{\Lambda}\)
    Mat33 sqrt_info_acc_;   ///< Accel bias \(\sqrt{\Lambda}\)
};

/**
 * @struct autonomy::localization::atlas::backend::ImuBiasPriorCostFunctor
 * @brief Bias prior: \(r=\sqrt{\Lambda}(b-b_0)\) (ORB EdgePriorGyro/Acc).
 *
 * Residual dimension: 6. Parameter block: bias(6).
 */
struct ImuBiasPriorCostFunctor {
    /**
     * @brief Construct bias prior.
     * @param bg0 Gyro bias prior center.
     * @param ba0 Accel bias prior center.
     * @param sqrt_info_gyro Gyro prior weight.
     * @param sqrt_info_acc Accel prior weight.
     */
    ImuBiasPriorCostFunctor(const Vec3& bg0, const Vec3& ba0,
                            double sqrt_info_gyro, double sqrt_info_acc)
        : bg0_(bg0),
          ba0_(ba0),
          sqrt_info_gyro_(sqrt_info_gyro),
          sqrt_info_acc_(sqrt_info_acc) {}

    /**
     * @brief AutoDiff residual evaluation.
     * @tparam T Scalar type.
     * @param bias Bias parameter `[bg, ba]`.
     * @param[out] residuals Length-6 residual.
     * @return Always true.
     */
    template <typename T>
    bool operator()(const T* const bias, T* residuals) const {
        for (int k = 0; k < 3; ++k) {
            residuals[k] = T(sqrt_info_gyro_) * (bias[k] - T(bg0_[k]));
            residuals[k + 3] =
                T(sqrt_info_acc_) * (bias[k + 3] - T(ba0_[k]));
        }
        return true;
    }

    /**
     * @brief Factory: residual 6; block 6.
     * @param bg0 Gyro bias prior center.
     * @param ba0 Accel bias prior center.
     * @param sqrt_info_gyro Gyro prior weight.
     * @param sqrt_info_acc Accel prior weight.
     * @return Heap-allocated CostFunction.
     */
    static ceres::CostFunction* Create(const Vec3& bg0, const Vec3& ba0,
                                       double sqrt_info_gyro,
                                       double sqrt_info_acc) {
        return new ceres::AutoDiffCostFunction<ImuBiasPriorCostFunctor, 6, 6>(
            new ImuBiasPriorCostFunctor(bg0, ba0, sqrt_info_gyro,
                                        sqrt_info_acc));
    }

    Vec3 bg0_;               ///< Gyro bias prior center
    Vec3 ba0_;               ///< Accel bias prior center
    double sqrt_info_gyro_;  ///< Gyro prior weight
    double sqrt_info_acc_;   ///< Accel prior weight
};

/**
 * @struct autonomy::localization::atlas::backend::ImuPreintegrationGSCostFunctor
 * @brief IMU residual with gravity direction and scale (ORB EdgeInertialGS,
 *        visual-inertial initialization).
 *
 * Poses \(T_{wb}\) are baked into members (fixed). Optimized parameters:
 * `vel_i(3), vel_j(3), bias(6), Rwg_aa(3), scale(1)`.
 * \(g = R_{wg}(0,0,-g)\); position/velocity differences are scaled by \(s\).
 *
 * Residual dimension: 9.
 */
struct ImuPreintegrationGSCostFunctor {
    /**
     * @brief Construct; poses are taken from the two keyframes' \(T_{wb}\).
     * @param pre Preintegrator.
     * @param Twb_i Body pose at keyframe i (fixed).
     * @param Twb_j Body pose at keyframe j (fixed).
     * @param sqrt_info 9×9 \(\sqrt{\Lambda}\).
     */
    ImuPreintegrationGSCostFunctor(const sensor::imu::Preintegrator& pre,
                                   const SE3& Twb_i, const SE3& Twb_j,
                                   const Eigen::Matrix<double, 9, 9>& sqrt_info)
        : dR_(pre.dR),
          dV_(pre.dV),
          dP_(pre.dP),
          JRg_(pre.JRg),
          JVg_(pre.JVg),
          JVa_(pre.JVa),
          JPg_(pre.JPg),
          JPa_(pre.JPa),
          bg0_(pre.bias.gyroscope),
          ba0_(pre.bias.accelerometer),
          dt_(pre.delta_t),
          Ri_(Twb_i.rotation()),
          ti_(Twb_i.translation()),
          Rj_(Twb_j.rotation()),
          tj_(Twb_j.translation()),
          sqrt_info_(sqrt_info) {}

    /**
     * @brief AutoDiff: parameters vel_i, vel_j, bias, rwg_aa, scale.
     * @tparam T Scalar type.
     * @param vel_i Velocity at keyframe i.
     * @param vel_j Velocity at keyframe j.
     * @param bias Bias `[bg, ba]`.
     * @param rwg_aa Gravity-frame rotation as angle-axis.
     * @param scale Scale parameter (length 1).
     * @param[out] residuals Length-9 residual.
     * @return Always true.
     */
    template <typename T>
    bool operator()(const T* const vel_i, const T* const vel_j,
                    const T* const bias, const T* const rwg_aa,
                    const T* const scale, T* residuals) const {
        Eigen::Matrix<T, 3, 3> Rwg;
        ceres::AngleAxisToRotationMatrix(rwg_aa, Rwg.data());
        const Eigen::Matrix<T, 3, 1> gI(T(0), T(0), T(-sensor::imu::kGravity));
        const Eigen::Matrix<T, 3, 1> g = Rwg * gI;
        const T s = scale[0];

        const Eigen::Matrix<T, 3, 1> dbg(bias[0] - T(bg0_[0]),
                                        bias[1] - T(bg0_[1]),
                                        bias[2] - T(bg0_[2]));
        const Eigen::Matrix<T, 3, 1> dba(bias[3] - T(ba0_[0]),
                                        bias[4] - T(ba0_[1]),
                                        bias[5] - T(ba0_[2]));
        const Eigen::Matrix<T, 3, 1> aa = JRg_.cast<T>() * dbg;
        T dR_delta[9];
        ceres::AngleAxisToRotationMatrix(aa.data(), dR_delta);
        Eigen::Map<const Eigen::Matrix<T, 3, 3>> dR_delta_m(dR_delta);
        const Eigen::Matrix<T, 3, 3> dR_corr = dR_.cast<T>() * dR_delta_m;
        const Eigen::Matrix<T, 3, 1> dV_corr =
            dV_.cast<T>() + JVg_.cast<T>() * dbg + JVa_.cast<T>() * dba;
        const Eigen::Matrix<T, 3, 1> dP_corr =
            dP_.cast<T>() + JPg_.cast<T>() * dbg + JPa_.cast<T>() * dba;

        const Eigen::Matrix<T, 3, 3> Ri = Ri_.cast<T>();
        const Eigen::Matrix<T, 3, 3> Rj = Rj_.cast<T>();
        const Eigen::Matrix<T, 3, 1> ti = ti_.cast<T>();
        const Eigen::Matrix<T, 3, 1> tj = tj_.cast<T>();
        const Eigen::Matrix<T, 3, 1> vi(vel_i[0], vel_i[1], vel_i[2]);
        const Eigen::Matrix<T, 3, 1> vj(vel_j[0], vel_j[1], vel_j[2]);
        const T dt = T(dt_);

        const Eigen::Matrix<T, 3, 3> R_err =
            dR_corr.transpose() * Ri.transpose() * Rj;
        T R_err_aa[3];
        ceres::RotationMatrixToAngleAxis(R_err.data(), R_err_aa);
        const Eigen::Matrix<T, 3, 1> v_err =
            Ri.transpose() * (s * (vj - vi) - g * dt) - dV_corr;
        const Eigen::Matrix<T, 3, 1> p_err =
            Ri.transpose() * (s * (tj - ti - vi * dt) - T(0.5) * g * dt * dt) -
            dP_corr;

        Eigen::Matrix<T, 9, 1> e;
        e << R_err_aa[0], R_err_aa[1], R_err_aa[2], v_err[0], v_err[1],
            v_err[2], p_err[0], p_err[1], p_err[2];
        const Eigen::Matrix<T, 9, 1> we = sqrt_info_.cast<T>() * e;
        for (int k = 0; k < 9; ++k) {
            residuals[k] = we[k];
        }
        return true;
    }

    /**
     * @brief Factory: residual 9; blocks 3,3,6,3,1.
     * @param pre Preintegrator.
     * @param Twb_i Body pose at keyframe i.
     * @param Twb_j Body pose at keyframe j.
     * @return Heap-allocated CostFunction.
     */
    static ceres::CostFunction* Create(const sensor::imu::Preintegrator& pre,
                                       const SE3& Twb_i, const SE3& Twb_j) {
        return new ceres::AutoDiffCostFunction<ImuPreintegrationGSCostFunctor, 9,
                                               3, 3, 6, 3, 1>(
            new ImuPreintegrationGSCostFunctor(
                pre, Twb_i, Twb_j,
                sensor::imu::ComputeSqrtInformation9(pre)));
    }

    Mat33 dR_, JRg_, JVg_, JVa_, JPg_, JPa_, Ri_, Rj_;
    Vec3 dV_, dP_, bg0_, ba0_, ti_, tj_;
    double dt_;
    Eigen::Matrix<double, 9, 9> sqrt_info_;
};

/**
 * @struct autonomy::localization::atlas::backend::PoseImuPriorCostFunctor
 * @brief 15-D prior on \(T_{wb}\), velocity and bias (ORB `EdgePriorPoseImu`).
 *
 * Residual order: \(\mathrm{Log}(R^\top \hat R)\), \(R^\top(\hat t-t)\),
 * \(\hat v-v\), \(\hat b_g-b_g\), \(\hat b_a-b_a\).
 * Ceres dimensions: residual 15; blocks 6, 3, 6 (pose, velocity, bias).
 */
struct PoseImuPriorCostFunctor {
    /**
     * @brief Construct from a prior mean and its square-root information.
     * @param Twb Prior body pose.
     * @param velocity Prior world velocity.
     * @param bias Prior IMU bias.
     * @param sqrt_info 15×15 square-root information.
     */
    PoseImuPriorCostFunctor(const SE3& Twb, const Vec3& velocity,
                            const sensor::imu::Bias& bias,
                            const Eigen::Matrix<double, 15, 15>& sqrt_info)
        : R_(Twb.rotation()),
          t_(Twb.translation()),
          velocity_(velocity),
          gyro_(bias.gyroscope),
          acc_(bias.accelerometer),
          sqrt_info_(sqrt_info) {}

    /**
     * @brief Evaluate the whitened 15-D residual.
     * @param pose `[angle_axis(3), translation(3)]` for \(T_{wb}\).
     * @param velocity World velocity.
     * @param bias `[bg(3), ba(3)]`.
     * @param residuals Output residual.
     * @return true.
     */
    template <typename T>
    bool operator()(const T* const pose, const T* const velocity,
                    const T* const bias, T* residuals) const {
        Eigen::Matrix<T, 3, 3> R_hat;
        ceres::AngleAxisToRotationMatrix(pose, R_hat.data());
        const Eigen::Matrix<T, 3, 1> t_hat(pose[3], pose[4], pose[5]);
        const Eigen::Matrix<T, 3, 3> R_err = R_.cast<T>().transpose() * R_hat;
        T aa[3];
        ceres::RotationMatrixToAngleAxis(R_err.data(), aa);
        const Eigen::Matrix<T, 3, 1> et =
            R_.cast<T>().transpose() * (t_hat - t_.cast<T>());
        Eigen::Matrix<T, 15, 1> e;
        e << aa[0], aa[1], aa[2], et[0], et[1], et[2],
            velocity[0] - T(velocity_.x()), velocity[1] - T(velocity_.y()),
            velocity[2] - T(velocity_.z()), bias[0] - T(gyro_.x()),
            bias[1] - T(gyro_.y()), bias[2] - T(gyro_.z()),
            bias[3] - T(acc_.x()), bias[4] - T(acc_.y()), bias[5] - T(acc_.z());
        const Eigen::Matrix<T, 15, 1> we = sqrt_info_.cast<T>() * e;
        for (int k = 0; k < 15; ++k) {
            residuals[k] = we[k];
        }
        return true;
    }

    /**
     * @brief Factory: residual 15; blocks 6, 3, 6.
     * @param Twb Prior body pose.
     * @param velocity Prior world velocity.
     * @param bias Prior IMU bias.
     * @param sqrt_info 15×15 square-root information.
     * @return Heap-allocated CostFunction.
     */
    static ceres::CostFunction* Create(
        const SE3& Twb, const Vec3& velocity, const sensor::imu::Bias& bias,
        const Eigen::Matrix<double, 15, 15>& sqrt_info) {
        return new ceres::AutoDiffCostFunction<PoseImuPriorCostFunctor, 15, 6,
                                               3, 6>(
            new PoseImuPriorCostFunctor(Twb, velocity, bias, sqrt_info));
    }

    /**
     * @brief Diagonal square-root information used when the marginalized
     *        Hessian is not available.
     * @return 15×15 diagonal matrix (rotation, translation, velocity, gyro, acc).
     */
    static Eigen::Matrix<double, 15, 15> DiagonalSqrtInformation() {
        Eigen::Matrix<double, 15, 1> diag;
        diag << 10, 10, 10, 10, 10, 10, 10, 10, 10, 1e2, 1e2, 1e2, 10, 10, 10;
        return diag.asDiagonal();
    }

    Mat33 R_ = Mat33::Identity();
    Vec3 t_ = Vec3::Zero();
    Vec3 velocity_ = Vec3::Zero();
    Vec3 gyro_ = Vec3::Zero();
    Vec3 acc_ = Vec3::Zero();
    Eigen::Matrix<double, 15, 15> sqrt_info_ =
        Eigen::Matrix<double, 15, 15>::Identity();
};

}  // namespace backend
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_BACKEND_COST_FUNCTIONS_IMU_COST_FUNCTION_HPP_

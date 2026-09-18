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

#include "autonomy/localization/atlas/frontend/eskf/anderson_acceleration.hpp"
#include "autonomy/localization/atlas/type.hpp"
#include "autonomy/localization/atlas/util/constant.hpp"

namespace autonomy::localization::atlas {
namespace estimate {
struct LidarFactorBatch;
}  // namespace estimate

namespace frontend {

/**
 * Error-state Kalman filter for pure LO/LIO iterate strategy.
 * Path: frontend/eskf/; namespace: frontend.
 * Not a second SLAM process; pose writes go via LocalEstimator.
 *
 * State error order (15): δθ(3), δp(3), δv(3), δba(3), δbg(3).
 *
 * IEKF UpdateLidar: iterated point-plane with analytic H on 6-DoF pose,
 * information-form solve on pose block + Joseph on pose subspace.
 * Optional Anderson Acceleration + per-iter dx clip (lightning-style).
 * PredictImu: discrete Φ ≈ I + F dt, P ← Φ P Φᵀ + G Q Gᵀ dt.
 * Degeneracy: eigenvalue projector on HTH (lightning-style), optional P inflate.
 */
class Eskf {
public:
    static constexpr int kDim = 15;
    using MatP_t = Eigen::Matrix<double, kDim, kDim>;
    using VecP_t = Eigen::Matrix<double, kDim, 1>;

    struct State {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Mat44_t T_wb = Mat44_t::Identity();  // body in world
        Vec3_t v = Vec3_t::Zero();
        Vec3_t ba = Vec3_t::Zero();
        Vec3_t bg = Vec3_t::Zero();
        //! World-frame gravity used by PredictImu (default ENU-down ≈ −gẑ).
        Vec3_t gravity = Vec3_t(0.0, 0.0, -util::constant::kGRAVITY);
    };

    struct Options {
        double gyro_noise = 1e-3;
        double acc_noise = 1e-2;
        double gyro_bias_noise = 1e-5;
        double acc_bias_noise = 1e-4;
        double lidar_noise = 0.1;
        //! Max IEKF iterations for UpdateLidar (analytic H).
        int max_iekf_iter = 4;
        //! Legacy flag: kept for compat; IEKF always updates P via Joseph.
        bool joseph_lidar_update = true;
        double iekf_converge_eps = 1e-5;
        //! Mild P inflation after each PredictImu (lightning predict_cov_inflation).
        double predict_cov_inflation = 1.01;
        //! Project / inflate unobservable pose DOF from HTH eigendecomposition.
        bool enable_degeneracy_check = true;
        //! Mark DOF degenerate if λ_i / λ_max < this ratio.
        double degeneracy_eigen_thresh = 0.01;
        //! Extra pose-block P scale when any DOF is degenerate.
        double degeneracy_cov_inflation = 1.02;
        //! Anderson Acceleration on 6-DoF dx (default off, lightning use_aa_).
        bool enable_anderson = false;
        //! Reject IEKF iter if |Δt| or |Δθ| exceeds these (lightning clip).
        double max_update_translation_step = 0.5;       // meters
        double max_update_rotation_step_deg = 5.0;      // degrees
    };

    Eskf() = default;
    explicit Eskf(Options options) : options_(std::move(options)) {
        ResetCovariance();
    }

    void set_options(Options options) { options_ = std::move(options); }
    [[nodiscard]] const Options& options() const { return options_; }

    void Reset(const Mat44_t& T_wb = Mat44_t::Identity());
    void PredictImu(double dt, const Vec3_t& gyro, const Vec3_t& acc);

    //! Body-frame relative odom: T_wb ← T_wb * T_delta (WIO / LWIO).
    void UpdateOdom(const Mat44_t& T_delta);

    //! Iterated EKF point-plane update (analytic H on δθ, δp).
    int UpdateLidar(const estimate::LidarFactorBatch& batch);

    void set_gyro_bias(const Vec3_t& bg) { state_.bg = bg; }
    void set_acc_bias(const Vec3_t& ba) { state_.ba = ba; }
    void set_gravity(const Vec3_t& g) { state_.gravity = g; }
    //! After IMUInit: shrink bg uncertainty (lightning ChangeP on bg block).
    void SetImuInitCovariance();

    [[nodiscard]] const State& state() const { return state_; }
    [[nodiscard]] State& mutable_state() { return state_; }
    [[nodiscard]] Mat44_t T_wb() const { return state_.T_wb; }
    [[nodiscard]] const MatP_t& covariance() const { return P_; }
    [[nodiscard]] MatP_t& mutable_covariance() { return P_; }

private:
    void ResetCovariance();
    void PropagateCovariance(double dt, const Vec3_t& omega, const Vec3_t& acc);

    Options options_;
    State state_;
    MatP_t P_ = MatP_t::Identity();
    bool initialized_ = false;
    AndersonAcceleration<double, 6, 10> aa_;
};

}  // namespace frontend
}  // namespace autonomy::localization::atlas

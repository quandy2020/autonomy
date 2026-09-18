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

#include "autonomy/localization/atlas/estimate/lidar_residual_source.hpp"
#include "autonomy/localization/atlas/type.hpp"

namespace autonomy::localization::atlas {
namespace frontend {

/**
 * Error-state Kalman filter for pure LO/LIO iterate strategy.
 * Path: frontend/eskf/; namespace: frontend.
 * Not a second SLAM process; pose writes go via LocalEstimator.
 *
 * State error order (15): δθ(3), δp(3), δv(3), δba(3), δbg(3).
 *
 * Gap vs lightning-lm full ESKF (remaining):
 *  - No IEKF / iterated Kalman update with analytic H from point-plane;
 *    pose still from Gauss-Newton, then a coarse Joseph scalar on pose block.
 *  - No gravity / extrinsic in-state, no 18/24-dim variants, no manifold
 *    boxplus reset of full P after every update.
 *  - Process noise is diagonal continuous Q*dt (not IMU discrete ΦQΦᵀ).
 *  - Bias random-walk only; no online bias observability from lidar alone.
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
    };

    struct Options {
        double gyro_noise = 1e-3;
        double acc_noise = 1e-2;
        double gyro_bias_noise = 1e-5;
        double acc_bias_noise = 1e-4;
        double lidar_noise = 0.1;
        //! After GN pose update, run a coarse Joseph scalar update on P.
        bool joseph_lidar_update = true;
    };

    Eskf() = default;
    explicit Eskf(Options options) : options_(std::move(options)) {
        ResetCovariance();
    }

    void Reset(const Mat44_t& T_wb = Mat44_t::Identity());
    void PredictImu(double dt, const Vec3_t& gyro, const Vec3_t& acc);

    //! Body-frame relative odom: T_wb ← T_wb * T_delta (WIO / LWIO).
    void UpdateOdom(const Mat44_t& T_delta);

    //! Point-plane update using ObsModel residual batch (body points + world planes).
    int UpdateLidar(const estimate::LidarFactorBatch& batch);

    [[nodiscard]] const State& state() const { return state_; }
    [[nodiscard]] Mat44_t T_wb() const { return state_.T_wb; }
    [[nodiscard]] const MatP_t& covariance() const { return P_; }

private:
    void ResetCovariance();
    void PropagateCovariance(double dt);
    void JosephPoseUpdate(double mean_residual);

    Options options_;
    State state_;
    MatP_t P_ = MatP_t::Identity();
    bool initialized_ = false;
};

}  // namespace frontend
}  // namespace autonomy::localization::atlas

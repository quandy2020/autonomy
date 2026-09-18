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
namespace sensor {
namespace lightning {

/**
 * Error-state Kalman filter skeleton for pure LO/LIO iterate strategy.
 * Lives under sensor/lidar — not a second SLAM process; pose writes go back
 * into Atlas Frontend / Mapping via LocalEstimator.
 */
class Eskf {
public:
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
        double lidar_noise = 0.1;
    };

    Eskf() = default;
    explicit Eskf(Options options) : options_(std::move(options)) {}

    void Reset(const Mat44_t& T_wb = Mat44_t::Identity());
    void PredictImu(double dt, const Vec3_t& gyro, const Vec3_t& acc);

    //! Body-frame relative odom: T_wb ← T_wb * T_delta (WIO / LWIO).
    void UpdateOdom(const Mat44_t& T_delta);

    //! Point-plane update using ObsModel residual batch (body points + world planes).
    int UpdateLidar(const estimate::LidarFactorBatch& batch);

    [[nodiscard]] const State& state() const { return state_; }
    [[nodiscard]] Mat44_t T_wb() const { return state_.T_wb; }

private:
    Options options_;
    State state_;
    bool initialized_ = false;
};

}  // namespace lightning
}  // namespace sensor
}  // namespace autonomy::localization::atlas

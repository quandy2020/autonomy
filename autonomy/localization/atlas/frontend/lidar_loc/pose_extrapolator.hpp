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

//! High-rate pose extrapolation between lidar loc updates.
//! Constant-velocity by default; optional IMU predict via LocalEstimator.

#include "autonomy/localization/atlas/frontend/local_estimator.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <mutex>

namespace autonomy::localization::atlas {
namespace frontend {

class PoseExtrapolator {
public:
    struct Options {
        //! Prefer LocalEstimator::PredictImu when set; else constant-velocity.
        bool use_imu_predict = true;
    };

    PoseExtrapolator() = default;
    explicit PoseExtrapolator(Options options) : options_(std::move(options)) {}

    void set_options(Options options) { options_ = std::move(options); }
    void set_local_estimator(LocalEstimator* est) { estimator_ = est; }

    //! Seed / correct with a lidar (or NDT) pose at stamp `t_sec`.
    void SetLidarPose(double t_sec, const Mat44_t& T_wb);

    //! Constant-velocity or IMU propagation to `t_sec`.
    [[nodiscard]] Mat44_t PoseAt(double t_sec) const;

    //! Feed IMU for high-rate predict (when estimator_ is set).
    void PredictImu(double dt, const Vec3_t& gyro, const Vec3_t& acc);

    [[nodiscard]] bool initialized() const { return initialized_; }
    [[nodiscard]] Mat44_t last_pose() const;

private:
    Options options_;
    LocalEstimator* estimator_ = nullptr;
    mutable std::mutex mtx_;
    bool initialized_ = false;
    double t_lidar_ = 0.0;
    Mat44_t T_wb_ = Mat44_t::Identity();
    Vec3_t v_ = Vec3_t::Zero();
};

}  // namespace frontend
}  // namespace autonomy::localization::atlas

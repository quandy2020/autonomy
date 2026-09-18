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
        //! Local IMU integrate on T_pred_ for high-rate viz (does NOT call
        //! LocalEstimator::PredictImu — lidar BuildImuPoses owns that).
        bool use_imu_predict = true;
        //! 0 = raw extrapolated pose; toward 1 blends toward last lidar pose
        //! (translation lerp + SO3 slerp). Applied in PoseAt / PoseCwAt.
        //! Default >0 reduces standstill TF spin from IMU noise.
        double smooth_factor = 0.5;
        //! |gyro| below this (rad/s) treated as zero in PredictImu.
        double gyro_static_thresh = 0.02;
        //! Snap LocalEstimator pose on SetLidarPose. Default OFF: LIO already
        //! wrote T_wb via UpdateLidar; syncing with zero_velocity wiped |v|
        //! every scan → IMU predict dead → IEKF-only zigzag.
        bool sync_estimator_on_lidar = false;
    };

    PoseExtrapolator() = default;
    explicit PoseExtrapolator(Options options) : options_(std::move(options)) {}

    void set_options(Options options) { options_ = std::move(options); }
    [[nodiscard]] const Options& options() const { return options_; }
    void set_local_estimator(LocalEstimator* est) { estimator_ = est; }

    //! Seed / correct with a lidar (or NDT) pose at stamp `t_sec`.
    //! Updates smooth anchor only — does not Reset LocalEstimator.
    void SetLidarPose(double t_sec, const Mat44_t& T_wb);

    //! Predicted pose at `t_sec`, optionally smoothed toward last lidar.
    [[nodiscard]] Mat44_t PoseAt(double t_sec) const;

    //! Publish helper: PoseAt(t).inverse() for VizBridge::PublishWorldPose.
    [[nodiscard]] Mat44_t PoseCwAt(double t_sec) const;

    //! Feed IMU for high-rate local predict (viz only; no estimator mutate).
    void PredictImu(double dt, const Vec3_t& gyro, const Vec3_t& acc);

    [[nodiscard]] bool initialized() const { return initialized_; }
    [[nodiscard]] Mat44_t last_pose() const;
    [[nodiscard]] Mat44_t last_lidar_pose() const;

private:
    [[nodiscard]] Mat44_t RawPoseAt(double t_sec) const;
    [[nodiscard]] Mat44_t BlendTowardLidar(const Mat44_t& T_pred) const;

    Options options_;
    LocalEstimator* estimator_ = nullptr;
    mutable std::mutex mtx_;
    bool initialized_ = false;
    double t_lidar_ = 0.0;
    double t_pred_ = 0.0;
    Mat44_t T_lidar_ = Mat44_t::Identity();
    //! Constant-velocity prediction state (also synced on SetLidarPose).
    Mat44_t T_pred_ = Mat44_t::Identity();
    Vec3_t v_ = Vec3_t::Zero();
};

}  // namespace frontend
}  // namespace autonomy::localization::atlas

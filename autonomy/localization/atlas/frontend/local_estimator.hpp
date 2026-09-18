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

//! frontend/local_estimator — realtime pose (ESKF for LO/LIO strategy).
//! Lives in Frontend; writes the same Atlas pose State (not a second SLAM).

#include "autonomy/localization/atlas/frontend/eskf/eskf.hpp"
#include "autonomy/localization/atlas/type.hpp"

namespace autonomy::localization::atlas {
namespace estimate {
struct LidarFactorBatch;
}  // namespace estimate

namespace frontend {

class LocalEstimator {
public:
    LocalEstimator() = default;
    explicit LocalEstimator(frontend::Eskf::Options eskf_opts)
        : eskf_(std::move(eskf_opts)) {}

    void Reset(const Mat44_t& T_wb = Mat44_t::Identity()) { eskf_.Reset(T_wb); }

    //! Lidar / NDT anchor: keep biases, snap T_wb (Lightning-style correction).
    void SetPose(const Mat44_t& T_wb, bool zero_velocity = true) {
        eskf_.SetPose(T_wb, zero_velocity);
    }

    void PredictImu(double dt, const Vec3_t& gyro, const Vec3_t& acc) {
        eskf_.PredictImu(dt, gyro, acc);
    }

    //! Compose body-frame odom delta into world pose (needed for WIO / LWIO).
    void UpdateOdom(const Mat44_t& T_delta) { eskf_.UpdateOdom(T_delta); }

    int UpdateLidar(const estimate::LidarFactorBatch& batch);

    void SetT_imu_lidar(const Mat44_t& T_il) { T_imu_lidar_ = T_il; }
    [[nodiscard]] const Mat44_t& T_imu_lidar() const { return T_imu_lidar_; }

    void set_eskf_options(frontend::Eskf::Options opts) {
        eskf_.set_options(std::move(opts));
    }
    [[nodiscard]] const frontend::Eskf::Options& eskf_options() const {
        return eskf_.options();
    }

    void set_gyro_bias(const Vec3_t& bg) { eskf_.set_gyro_bias(bg); }
    void set_acc_bias(const Vec3_t& ba) { eskf_.set_acc_bias(ba); }
    void set_gravity(const Vec3_t& g) { eskf_.set_gravity(g); }
    void set_velocity(const Vec3_t& v) { eskf_.mutable_state().v = v; }
    void SetImuInitCovariance() { eskf_.SetImuInitCovariance(); }

    [[nodiscard]] Mat44_t T_wb() const { return eskf_.T_wb(); }
    [[nodiscard]] Mat44_t T_cw() const { return eskf_.T_wb().inverse(); }
    [[nodiscard]] Vec3_t velocity() const { return eskf_.state().v; }
    [[nodiscard]] Vec3_t gravity() const { return eskf_.state().gravity; }
    [[nodiscard]] const frontend::Eskf::State& state() const {
        return eskf_.state();
    }
    [[nodiscard]] const frontend::Eskf::State& nav_state() const {
        return eskf_.state();
    }
    [[nodiscard]] const frontend::Eskf::MatP_t& covariance() const {
        return eskf_.covariance();
    }

private:
    frontend::Eskf eskf_;
    Mat44_t T_imu_lidar_ = Mat44_t::Identity();
};

}  // namespace frontend
}  // namespace autonomy::localization::atlas

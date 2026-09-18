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

//! frontend/lio/imu_process — IMU predict + trajectory deskew (lightning UndistortPcl ideas).

#include "autonomy/localization/atlas/frontend/lio/measure_group.hpp"
#include "autonomy/localization/atlas/frontend/local_estimator.hpp"
#include "autonomy/localization/atlas/sensor/types.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <cmath>
#include <deque>
#include <vector>

namespace autonomy::localization::atlas {
namespace frontend {
namespace lio {

//! One IMU-propagated pose sample along a lidar scan (offset from scan begin).
struct ImuPoseSample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double offset_t = 0.0;  // sec from scan begin
    Mat33_t R = Mat33_t::Identity();
    Vec3_t pos = Vec3_t::Zero();
    Vec3_t vel = Vec3_t::Zero();
    Vec3_t angvel = Vec3_t::Zero();
    Vec3_t acc = Vec3_t::Zero();
};

//! Forward-integrate IMU over measure, filling imu_poses; also PredictImu on estimator.
void BuildImuPoses(LocalEstimator* est,
                   const MeasureGroup& meas,
                   std::vector<ImuPoseSample>* imu_poses);

//! Backward deskew: interpolate pose at point time vs end pose;
//! p_end = R_end^T * (R_i * p + t_i - t_end) (with optional T_imu_lidar).
//! point_time_rel is [0,1] fraction of scan (or absolute offset sec if >1 — clamped).
std::vector<Vec3_t> UndistortByImuTrajectory(
    const std::vector<Vec3_t>& points_body,
    const std::vector<double>& point_time_rel,
    const std::vector<ImuPoseSample>& imu_poses,
    const Mat44_t& T_imu_lidar = Mat44_t::Identity());

//! Synthesize uniform t_rel = i/(n-1) when n>1 (approximation when driver has no times).
inline std::vector<double> SynthesizeUniformTimeRel(std::size_t n) {
    std::vector<double> t;
    if (n == 0) {
        return t;
    }
    if (n == 1) {
        t.push_back(0.0);
        return t;
    }
    t.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        t[i] = static_cast<double>(i) / static_cast<double>(n - 1);
    }
    return t;
}

class ImuProcess {
public:
    void SetExtrinsic(const Mat44_t& T_il = Mat44_t::Identity()) {
        T_imu_lidar_ = T_il;
    }

    [[nodiscard]] const Mat44_t& T_imu_lidar() const { return T_imu_lidar_; }

    //! Propagate LocalEstimator with successive IMU samples (uses sample Δt).
    void ProcessPredict(LocalEstimator* estimator,
                        const std::deque<sensor::ImuSample>& imu) const {
        if (!estimator || imu.size() < 2) {
            return;
        }
        for (std::size_t i = 1; i < imu.size(); ++i) {
            const double dt = imu[i].timestamp - imu[i - 1].timestamp;
            if (dt <= 0.0 || dt > 0.5) {
                continue;
            }
            estimator->PredictImu(dt, imu[i].gyro, imu[i].acc);
        }
    }

    //! Given imu queue + fixed dt, call estimator PredictImu once per sample.
    void ProcessPredict(LocalEstimator* estimator,
                        const std::deque<sensor::ImuSample>& imu,
                        double dt) const {
        if (!estimator || imu.empty() || dt <= 0.0) {
            return;
        }
        for (const auto& s : imu) {
            estimator->PredictImu(dt, s.gyro, s.acc);
        }
    }

    void BuildImuPoses(LocalEstimator* est,
                       const MeasureGroup& meas,
                       std::vector<ImuPoseSample>* imu_poses) const {
        lio::BuildImuPoses(est, meas, imu_poses);
    }

    [[nodiscard]] std::vector<Vec3_t> UndistortByImuTrajectory(
        const std::vector<Vec3_t>& points_body,
        const std::vector<double>& point_time_rel,
        const std::vector<ImuPoseSample>& imu_poses) const {
        return lio::UndistortByImuTrajectory(points_body, point_time_rel,
                                             imu_poses, T_imu_lidar_);
    }

    //! Constant-ω deskew fallback when imu_poses size < 2.
    //! If point_time_rel empty / size mismatch → no-op copy.
    [[nodiscard]] std::vector<Vec3_t> UndistortScan(
        const std::vector<Vec3_t>& points_body,
        const std::vector<double>& point_time_rel,
        const Mat44_t& /*T_begin*/,
        const Vec3_t& omega) const {
        if (points_body.empty() || point_time_rel.size() != points_body.size()) {
            return points_body;
        }
        const double wnorm = omega.norm();
        if (wnorm < 1e-12) {
            return points_body;
        }
        const Vec3_t axis = omega / wnorm;
        std::vector<Vec3_t> out;
        out.reserve(points_body.size());
        (void)T_imu_lidar_;
        for (std::size_t i = 0; i < points_body.size(); ++i) {
            double alpha = point_time_rel[i];
            if (!std::isfinite(alpha)) {
                out.push_back(points_body[i]);
                continue;
            }
            if (alpha < 0.0) {
                alpha = 0.0;
            } else if (alpha > 1.0) {
                alpha = 1.0;
            }
            const double angle = wnorm * alpha;
            const Mat33_t R_a =
                Eigen::AngleAxisd(angle, axis).toRotationMatrix();
            out.push_back(R_a * points_body[i]);
        }
        return out;
    }

private:
    //! Lidar in IMU frame extrinsic (T_imu_lidar: p_imu = R p_lidar + t).
    Mat44_t T_imu_lidar_ = Mat44_t::Identity();
};

}  // namespace lio
}  // namespace frontend
}  // namespace autonomy::localization::atlas

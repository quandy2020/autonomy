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

#include "autonomy/localization/atlas/frontend/lidar_loc/pose_extrapolator.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy::localization::atlas {
namespace frontend {
namespace {

double Clamp01(double x) {
    return std::max(0.0, std::min(1.0, x));
}

}  // namespace

void PoseExtrapolator::SetLidarPose(double t_sec, const Mat44_t& T_wb) {
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (initialized_ && t_sec > t_lidar_ + 1e-6) {
            const double dt = t_sec - t_lidar_;
            const Vec3_t dp =
                T_wb.block<3, 1>(0, 3) - T_lidar_.block<3, 1>(0, 3);
            constexpr double kMinTransForVelocity = 0.03;  // m
            if (estimator_ && estimator_->velocity().norm() > 1e-3) {
                // Prefer ESKF velocity (BuildImuPoses); finite-diff is backup.
                v_ = estimator_->velocity();
            } else if (dp.norm() < kMinTransForVelocity) {
                v_.setZero();
            } else {
                v_ = dp / std::max(dt, 1e-3);
            }
        } else if (estimator_ && estimator_->velocity().norm() > 1e-3) {
            v_ = estimator_->velocity();
        } else {
            v_.setZero();
        }
        t_lidar_ = t_sec;
        t_pred_ = t_sec;
        T_lidar_ = T_wb;
        T_pred_ = T_wb;
        initialized_ = true;
    }
    // Do not SetPose(zero_velocity) on estimator — LIO path already holds
    // T_wb; wiping v each scan forces IEKF-only motion and traj zigzag.
    if (options_.sync_estimator_on_lidar && estimator_) {
        estimator_->SetPose(T_wb, /*zero_velocity=*/false);
    }
}

Mat44_t PoseExtrapolator::RawPoseAt(double t_sec) const {
    if (!initialized_) {
        return Mat44_t::Identity();
    }
    // Always use local T_pred_ (advanced by PredictImu). Do not read
    // estimator_->T_wb() here — lidar deskew owns estimator PredictImu.
    const double dt = t_sec - t_pred_;
    Mat44_t T = T_pred_;
    T.block<3, 1>(0, 3) += v_ * dt;
    return T;
}

Mat44_t PoseExtrapolator::BlendTowardLidar(const Mat44_t& T_pred) const {
    const double s = Clamp01(options_.smooth_factor);
    if (s <= 1e-12) {
        return T_pred;
    }
    // T = slerp/lerp(T_lidar, T_pred, 1-s): s→1 pulls toward last lidar.
    const double alpha = 1.0 - s;
    const Quat_t q_l(T_lidar_.block<3, 3>(0, 0));
    const Quat_t q_p(T_pred.block<3, 3>(0, 0));
    const Quat_t q = q_l.slerp(alpha, q_p).normalized();
    Mat44_t T = Mat44_t::Identity();
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
    T.block<3, 1>(0, 3) =
        (1.0 - alpha) * T_lidar_.block<3, 1>(0, 3) +
        alpha * T_pred.block<3, 1>(0, 3);
    return T;
}

Mat44_t PoseExtrapolator::PoseAt(double t_sec) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return BlendTowardLidar(RawPoseAt(t_sec));
}

Mat44_t PoseExtrapolator::PoseCwAt(double t_sec) const {
    return PoseAt(t_sec).inverse();
}

void PoseExtrapolator::PredictImu(double dt, const Vec3_t& gyro,
                                  const Vec3_t& acc) {
    // High-rate viz only: advance local T_pred_. Never call
    // estimator_->PredictImu — BuildImuPoses owns that (no double integrate).
    (void)acc;  // Do not integrate body accel (includes gravity → zig-zag).
    Vec3_t g = gyro;
    if (g.norm() < options_.gyro_static_thresh) {
        g.setZero();
    }
    std::lock_guard<std::mutex> lock(mtx_);
    if (!initialized_ || dt <= 0.0) {
        return;
    }
    if (!options_.use_imu_predict) {
        T_pred_.block<3, 1>(0, 3) += v_ * dt;
        t_pred_ += dt;
        return;
    }
    const double ang = g.norm() * dt;
    if (ang > 1e-12) {
        T_pred_.block<3, 3>(0, 0) =
            T_pred_.block<3, 3>(0, 0) *
            Eigen::AngleAxisd(ang, g.normalized()).toRotationMatrix();
    }
    // Translation: constant velocity from last lidar (SetLidarPose), no accel.
    T_pred_.block<3, 1>(0, 3) += v_ * dt;
    t_pred_ += dt;
}

Mat44_t PoseExtrapolator::last_pose() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return T_pred_;
}

Mat44_t PoseExtrapolator::last_lidar_pose() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return T_lidar_;
}

}  // namespace frontend
}  // namespace autonomy::localization::atlas

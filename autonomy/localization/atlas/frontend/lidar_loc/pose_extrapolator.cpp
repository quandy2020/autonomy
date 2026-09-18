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

namespace autonomy::localization::atlas {
namespace frontend {

void PoseExtrapolator::SetLidarPose(double t_sec, const Mat44_t& T_wb) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (initialized_ && t_sec > t_lidar_ + 1e-6) {
        const double dt = t_sec - t_lidar_;
        const Vec3_t dp = T_wb.block<3, 1>(0, 3) - T_wb_.block<3, 1>(0, 3);
        v_ = dp / dt;
    }
    t_lidar_ = t_sec;
    T_wb_ = T_wb;
    initialized_ = true;
    if (estimator_) {
        estimator_->Reset(T_wb);
    }
}

Mat44_t PoseExtrapolator::PoseAt(double t_sec) const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!initialized_) {
        return Mat44_t::Identity();
    }
    if (options_.use_imu_predict && estimator_) {
        return estimator_->T_wb();
    }
    const double dt = t_sec - t_lidar_;
    Mat44_t T = T_wb_;
    T.block<3, 1>(0, 3) += v_ * dt;
    return T;
}

void PoseExtrapolator::PredictImu(double dt, const Vec3_t& gyro,
                                  const Vec3_t& acc) {
    if (estimator_ && options_.use_imu_predict) {
        estimator_->PredictImu(dt, gyro, acc);
        std::lock_guard<std::mutex> lock(mtx_);
        T_wb_ = estimator_->T_wb();
        v_ = estimator_->velocity();
        return;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    if (!initialized_ || dt <= 0.0) {
        return;
    }
    const double ang = gyro.norm() * dt;
    if (ang > 1e-12) {
        T_wb_.block<3, 3>(0, 0) =
            T_wb_.block<3, 3>(0, 0) *
            Eigen::AngleAxisd(ang, gyro.normalized()).toRotationMatrix();
    }
    T_wb_.block<3, 1>(0, 3) += v_ * dt + 0.5 * acc * dt * dt;
    v_ += acc * dt;
    t_lidar_ += dt;
}

Mat44_t PoseExtrapolator::last_pose() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return T_wb_;
}

}  // namespace frontend
}  // namespace autonomy::localization::atlas

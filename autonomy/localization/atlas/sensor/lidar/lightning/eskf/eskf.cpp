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

#include "autonomy/localization/atlas/sensor/lidar/lightning/eskf/eskf.hpp"

#include <cmath>

namespace autonomy::localization::atlas {
namespace sensor {
namespace lightning {

void Eskf::Reset(const Mat44_t& T_wb) {
    state_ = State{};
    state_.T_wb = T_wb;
    initialized_ = true;
}

void Eskf::UpdateOdom(const Mat44_t& T_delta) {
    if (!initialized_) {
        Reset(Mat44_t::Identity());
    }
    // Right-multiply: T_delta is body_{k-1} → body_k in the body frame.
    state_.T_wb = state_.T_wb * T_delta;
}

void Eskf::PredictImu(double dt, const Vec3_t& gyro, const Vec3_t& acc) {
    if (!initialized_) {
        Reset(Mat44_t::Identity());
    }
    if (dt <= 0.0) {
        return;
    }
    const Vec3_t omega = gyro - state_.bg;
    const double angle = omega.norm() * dt;
    Mat33_t dR = Mat33_t::Identity();
    if (angle > 1e-12) {
        const Vec3_t axis = omega.normalized();
        dR = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    }
    Mat33_t R = state_.T_wb.block<3, 3>(0, 0);
    Vec3_t t = state_.T_wb.block<3, 1>(0, 3);
    R = R * dR;
    const Vec3_t a_w = R * (acc - state_.ba) + Vec3_t(0.0, 0.0, -9.81);
    t += state_.v * dt + 0.5 * a_w * dt * dt;
    state_.v += a_w * dt;
    state_.T_wb.block<3, 3>(0, 0) = R;
    state_.T_wb.block<3, 1>(0, 3) = t;
    (void)options_;
}

int Eskf::UpdateLidar(const estimate::LidarFactorBatch& batch) {
    if (batch.point_planes.empty()) {
        return 0;
    }
    if (!initialized_) {
        Reset(Mat44_t::Identity());
    }
    // Lightweight Gauss-Newton on SE3 using point-plane residuals (same model
    // as Joint BA). Full 18-state EKF covariance lands with upstream.
    Mat33_t R = state_.T_wb.block<3, 3>(0, 0);
    Vec3_t t = state_.T_wb.block<3, 1>(0, 3);
    int used = 0;
    for (int iter = 0; iter < 3; ++iter) {
        Vec6_t Htr = Vec6_t::Zero();
        Mat66_t HtH = Mat66_t::Zero();
        used = 0;
        for (const auto& r : batch.point_planes) {
            const Vec3_t pw = R * r.point_body + t;
            const double e = r.normal_world.dot(pw) + r.d;
            const Vec3_t Rp = R * r.point_body;
            Vec6_t j;
            j.head<3>() = -Rp.cross(r.normal_world);
            j.tail<3>() = r.normal_world;
            const double w = std::max(1e-6, r.weight) /
                             std::max(1e-6, options_.lidar_noise);
            HtH += w * j * j.transpose();
            Htr += w * j * (-e);
            ++used;
        }
        if (used == 0) {
            break;
        }
        HtH.diagonal().array() += 1e-4;
        const Vec6_t dx = HtH.ldlt().solve(Htr);
        const double ang = dx.head<3>().norm();
        if (ang > 1e-12) {
            R = Eigen::AngleAxisd(ang, dx.head<3>().normalized()).toRotationMatrix() *
                R;
        }
        t += dx.tail<3>();
        if (dx.norm() < 1e-5) {
            break;
        }
    }
    state_.T_wb.block<3, 3>(0, 0) = R;
    state_.T_wb.block<3, 1>(0, 3) = t;
    return used;
}

}  // namespace lightning
}  // namespace sensor
}  // namespace autonomy::localization::atlas

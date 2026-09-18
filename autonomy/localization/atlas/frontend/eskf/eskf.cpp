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

#include "autonomy/localization/atlas/frontend/eskf/eskf.hpp"

#include "autonomy/localization/atlas/estimate/lidar_residual_source.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy::localization::atlas {
namespace frontend {
namespace {

Mat33_t Skew(const Vec3_t& v) {
    Mat33_t m;
    m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return m;
}

}  // namespace

void Eskf::ResetCovariance() {
    P_.setIdentity();
    P_.block<3, 3>(0, 0) *= 1e-4;   // rot
    P_.block<3, 3>(3, 3) *= 1e-2;   // pos
    P_.block<3, 3>(6, 6) *= 1e-2;   // vel
    P_.block<3, 3>(9, 9) *= 1e-4;   // ba
    P_.block<3, 3>(12, 12) *= 1e-4; // bg
}

void Eskf::Reset(const Mat44_t& T_wb) {
    state_ = State{};
    state_.T_wb = T_wb;
    ResetCovariance();
    initialized_ = true;
}

void Eskf::UpdateOdom(const Mat44_t& T_delta) {
    if (!initialized_) {
        Reset(Mat44_t::Identity());
    }
    state_.T_wb = state_.T_wb * T_delta;
    P_.block<3, 3>(0, 0) += Mat33_t::Identity() * 1e-6;
    P_.block<3, 3>(3, 3) += Mat33_t::Identity() * 1e-4;
}

void Eskf::PropagateCovariance(double dt, const Vec3_t& omega,
                               const Vec3_t& acc) {
    // Φ ≈ I + F dt  (error-state: δθ, δp, δv, δba, δbg)
    MatP_t Phi = MatP_t::Identity();
    const Mat33_t R = state_.T_wb.block<3, 3>(0, 0);
    const Vec3_t a_body = acc - state_.ba;
    const Vec3_t a_world = R * a_body;

    Phi.block<3, 3>(0, 0) -= Skew(omega) * dt;           // gyro → rot
    Phi.block<3, 3>(0, 12) = -Mat33_t::Identity() * dt;  // bg → rot
    Phi.block<3, 3>(3, 6) = Mat33_t::Identity() * dt;    // vel → pos
    Phi.block<3, 3>(6, 0) = -Skew(a_world) * dt;         // rot → vel
    Phi.block<3, 3>(6, 9) = -R * dt;                     // ba → vel

    P_ = Phi * P_ * Phi.transpose();

    // G Q Gᵀ dt — diagonal process noise on gyro/acc/bias channels.
    VecP_t q = VecP_t::Zero();
    q.segment<3>(0).setConstant(options_.gyro_noise * options_.gyro_noise);
    q.segment<3>(3).setConstant(options_.acc_noise * options_.acc_noise);
    q.segment<3>(6).setConstant(options_.acc_noise * options_.acc_noise);
    q.segment<3>(9).setConstant(options_.acc_bias_noise *
                                options_.acc_bias_noise);
    q.segment<3>(12).setConstant(options_.gyro_bias_noise *
                                 options_.gyro_bias_noise);
    P_ += q.asDiagonal() * dt;

    for (int i = 0; i < kDim; ++i) {
        if (P_(i, i) < 1e-12) {
            P_(i, i) = 1e-12;
        }
    }
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
    PropagateCovariance(dt, omega, acc);
}

int Eskf::UpdateLidar(const estimate::LidarFactorBatch& batch) {
    if (batch.point_planes.empty()) {
        return 0;
    }
    if (!initialized_) {
        Reset(Mat44_t::Identity());
    }

    Mat33_t R = state_.T_wb.block<3, 3>(0, 0);
    Vec3_t t = state_.T_wb.block<3, 1>(0, 3);
    int used = 0;
    const int max_iter = std::max(1, options_.max_iekf_iter);
    const double Rmeas =
        std::max(1e-6, options_.lidar_noise * options_.lidar_noise);

    for (int iter = 0; iter < max_iter; ++iter) {
        Mat66_t HTH = Mat66_t::Zero();
        Vec6_t HTr = Vec6_t::Zero();
        used = 0;
        for (const auto& r : batch.point_planes) {
            const Vec3_t pw = R * r.point_body + t;
            const double e = r.normal_world.dot(pw) + r.d;
            // Analytic H: J_θ = -n × (R p), J_p = n
            const Vec3_t Rp = R * r.point_body;
            Vec6_t j;
            j.head<3>() = -r.normal_world.cross(Rp);
            j.tail<3>() = r.normal_world;
            const double w =
                std::max(1e-6, r.weight) / Rmeas;
            HTH.noalias() += w * j * j.transpose();
            HTr.noalias() += w * j * (-e);
            ++used;
        }
        if (used == 0) {
            break;
        }

        // Information form on 6-DoF pose block: dx = (HTH + P_pose^{-1})^{-1} HTr
        Mat66_t P_pose = P_.block<6, 6>(0, 0);
        // Symmetrize for numerical stability.
        P_pose = 0.5 * (P_pose + P_pose.transpose());
        P_pose.diagonal().array() += 1e-9;
        HTH.diagonal().array() += 1e-9;

        const Mat66_t Pinv =
            P_pose.ldlt().solve(Mat66_t::Identity());
        const Mat66_t Info = HTH + Pinv;
        const Vec6_t dx = Info.ldlt().solve(HTr);
        const Mat66_t P_post = Info.ldlt().solve(Mat66_t::Identity());

        // boxplus on R, t
        const double ang = dx.head<3>().norm();
        if (ang > 1e-12) {
            R = Eigen::AngleAxisd(ang, dx.head<3>().normalized())
                    .toRotationMatrix() *
                R;
        }
        t += dx.tail<3>();

        // Joseph-style update on pose subspace of full P.
        if (options_.joseph_lidar_update) {
            MatP_t G = MatP_t::Identity();
            // KH ≈ P_post * HTH on pose dims
            G.block<6, 6>(0, 0) =
                Mat66_t::Identity() - P_post * HTH;
            P_ = G * P_ * G.transpose();
            P_.block<6, 6>(0, 0) = P_post;
            for (int i = 0; i < kDim; ++i) {
                if (P_(i, i) < 1e-12) {
                    P_(i, i) = 1e-12;
                }
            }
        }

        if (dx.norm() < options_.iekf_converge_eps) {
            break;
        }
    }

    state_.T_wb.block<3, 3>(0, 0) = R;
    state_.T_wb.block<3, 1>(0, 3) = t;
    return used;
}

}  // namespace frontend
}  // namespace autonomy::localization::atlas

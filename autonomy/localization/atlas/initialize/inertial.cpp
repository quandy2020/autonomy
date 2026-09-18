/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/localization/atlas/initialize/inertial.hpp"

#include "autonomy/localization/atlas/data/keyframe.hpp"
#include "autonomy/localization/atlas/data/landmark.hpp"
#include "autonomy/localization/atlas/data/map_database.hpp"

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include "autolink/common/log.hpp"

namespace autonomy::localization::atlas {
namespace initialize {

inertial::inertial(const imu::config& cfg)
    : cfg_(cfg),
      gravity_(0.0, 0.0, -cfg.gravity_magnitude),
      min_keyframes_(cfg.init_min_keyframes) {}

void inertial::reset() {
    state_ = state_t::NotReady;
    keyfrms_.clear();
    velocities_.clear();
    scale_ = 1.0;
    gravity_ = Vec3_t(0.0, 0.0, -cfg_.gravity_magnitude);
    bias_ = imu::bias{};
}

void inertial::add_keyframe(const std::shared_ptr<data::keyframe>& keyfrm) {
    if (!keyfrm) {
        return;
    }
    // Keep temporal order; skip duplicates.
    if (!keyfrms_.empty() && keyfrms_.back()->id_ == keyfrm->id_) {
        return;
    }
    keyfrms_.push_back(keyfrm);
    if (state_ == state_t::NotReady && keyfrms_.size() >= 2) {
        state_ = state_t::Estimating;
    }
}

bool inertial::try_initialize() {
    if (keyfrms_.size() < min_keyframes_) {
        return false;
    }
    state_ = state_t::Estimating;
    if (!estimate_inertial_parameters()) {
        state_ = state_t::Failed;
        return false;
    }
    refine_with_preintegration_ba();
    state_ = state_t::Succeeded;
    return true;
}

bool inertial::estimate_inertial_parameters() {
    // Linear MAP solve for [s, g, v_0..v_{N-1}] using consecutive preintegrations:
    //   R_i * Δv = v_j - v_i - g * Δt
    //   R_i * Δp = s*(t_j - t_i) - v_i*Δt - 0.5*g*Δt^2
    // (ORB-SLAM3 / Martinelli inertial-only initialization.)

    const Mat44_t T_c_b = cfg_.T_c_b();
    const std::size_t N = keyfrms_.size();
    if (N < 3) {
        return false;
    }

    // Count valid consecutive pairs.
    std::vector<std::size_t> pairs;
    pairs.reserve(N);
    for (std::size_t i = 0; i + 1 < N; ++i) {
        const auto pre = keyfrms_[i + 1]->get_imu_preintegrator();
        if (pre && pre->is_valid() && pre->delta_t() > 1e-3) {
            pairs.push_back(i);
        }
    }
    if (pairs.size() < 2) {
        return false;
    }

    // Unknowns: s(1) + g(3) + v_i(3N) = 4 + 3N
    const int n_vars = static_cast<int>(4 + 3 * N);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_vars, n_vars);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(n_vars);

    auto add_residual = [&](int row_base, const Eigen::MatrixXd& J, const Eigen::VectorXd& r,
                            double w) {
        // Accumulate normal equations J^T W J, J^T W r  (row_base unused; J is n_vars cols)
        (void)row_base;
        H.noalias() += w * J.transpose() * J;
        b.noalias() += w * J.transpose() * r;
    };

    for (const std::size_t i : pairs) {
        const auto& kf_i = keyfrms_[i];
        const auto& kf_j = keyfrms_[i + 1];
        const auto pre = kf_j->get_imu_preintegrator();
        const double dt = pre->delta_t();
        const Mat33_t R_i = kf_i->get_imu_rotation_wb(T_c_b);
        const Vec3_t t_i = kf_i->get_imu_translation_wb(T_c_b);
        const Vec3_t t_j = kf_j->get_imu_translation_wb(T_c_b);
        const Vec3_t dp_vis = t_j - t_i;

        // Velocity residual: R_i * Δv + g*dt + v_i - v_j = 0
        {
            Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, n_vars);
            Eigen::VectorXd r = -(R_i * pre->delta_v());
            // g
            J.block<3, 3>(0, 1) = Mat33_t::Identity() * dt;
            // v_i
            J.block<3, 3>(0, 4 + 3 * static_cast<int>(i)) = Mat33_t::Identity();
            // v_j
            J.block<3, 3>(0, 4 + 3 * static_cast<int>(i + 1)) = -Mat33_t::Identity();
            add_residual(0, J, r, 1.0);
        }

        // Position residual: R_i * Δp - s*dp_vis + v_i*dt + 0.5*g*dt^2 = 0
        {
            Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, n_vars);
            Eigen::VectorXd r = -(R_i * pre->delta_p());
            // s
            J.col(0) = -dp_vis;
            // g
            J.block<3, 3>(0, 1) = 0.5 * dt * dt * Mat33_t::Identity();
            // v_i
            J.block<3, 3>(0, 4 + 3 * static_cast<int>(i)) = dt * Mat33_t::Identity();
            add_residual(0, J, r, 1.0);
        }
    }

    // Prefer g ≈ (0,0,-mag)
    {
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, n_vars);
        J.block<3, 3>(0, 1) = Mat33_t::Identity();
        Eigen::VectorXd r = Vec3_t(0.0, 0.0, -cfg_.gravity_magnitude);
        add_residual(0, J, r, 0.5);
    }
    // Prefer s > 0 with mild prior around 1.
    {
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(1, n_vars);
        J(0, 0) = 1.0;
        Eigen::VectorXd r(1);
        r(0) = 1.0;
        add_residual(0, J, r, 1e-4);
    }

    Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
    if (ldlt.info() != Eigen::Success) {
        AWARN << "inertial init: normal equations failed";
        return false;
    }
    const Eigen::VectorXd x = ldlt.solve(b);
    if (!x.allFinite()) {
        return false;
    }

    scale_ = x(0);
    gravity_ = x.segment<3>(1);
    // Enforce gravity magnitude.
    if (gravity_.norm() > 1e-6) {
        gravity_ = gravity_.normalized() * cfg_.gravity_magnitude;
    } else {
        gravity_ = Vec3_t(0.0, 0.0, -cfg_.gravity_magnitude);
    }
    if (!std::isfinite(scale_) || scale_ < 1e-3 || scale_ > 100.0) {
        AWARN << "inertial init: invalid scale " << scale_;
        return false;
    }

    velocities_.assign(N, Vec3_t::Zero());
    for (std::size_t i = 0; i < N; ++i) {
        velocities_[i] = x.segment<3>(4 + 3 * static_cast<int>(i));
    }
    bias_ = imu::bias{};
    AINFO << "inertial init: scale=" << scale_ << " |g|=" << gravity_.norm()
          << " keyframes=" << N;
    return true;
}

bool inertial::refine_with_preintegration_ba() {
    // One Gauss-Newton refinement of velocities + gyro/acc bias with fixed scale/gravity.
    if (keyfrms_.size() < 3 || velocities_.size() != keyfrms_.size()) {
        return false;
    }
    const Mat44_t T_c_b = cfg_.T_c_b();
    const std::size_t N = keyfrms_.size();
    const int n_vars = static_cast<int>(3 * N + 6);  // v_i + ba + bg
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_vars, n_vars);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(n_vars);

    for (std::size_t i = 0; i + 1 < N; ++i) {
        const auto pre = keyfrms_[i + 1]->get_imu_preintegrator();
        if (!pre || !pre->is_valid()) {
            continue;
        }
        Mat33_t dR;
        Vec3_t dv, dp;
        pre->corrected_deltas(bias_, dR, dv, dp);
        const double dt = pre->delta_t();
        const Mat33_t R_i = keyfrms_[i]->get_imu_rotation_wb(T_c_b);
        const Vec3_t t_i = scale_ * keyfrms_[i]->get_imu_translation_wb(T_c_b);
        const Vec3_t t_j = scale_ * keyfrms_[i + 1]->get_imu_translation_wb(T_c_b);
        const Vec3_t& v_i = velocities_[i];
        const Vec3_t& v_j = velocities_[i + 1];

        const Vec3_t r_v = R_i.transpose() * (v_j - v_i - gravity_ * dt) - dv;
        const Vec3_t r_p = R_i.transpose() * (t_j - t_i - v_i * dt - 0.5 * gravity_ * dt * dt) - dp;

        // Analytic-ish Jacobians for v and bias (using preintegration jac).
        Eigen::MatrixXd Jv = Eigen::MatrixXd::Zero(3, n_vars);
        Jv.block<3, 3>(0, 3 * static_cast<int>(i)) = -R_i.transpose();
        Jv.block<3, 3>(0, 3 * static_cast<int>(i + 1)) = R_i.transpose();
        Jv.block<3, 3>(0, 3 * static_cast<int>(N)) = -pre->jacobian().block<3, 3>(3, 0);      // ba
        Jv.block<3, 3>(0, 3 * static_cast<int>(N) + 3) = -pre->jacobian().block<3, 3>(3, 3);  // bg
        H += Jv.transpose() * Jv;
        b += Jv.transpose() * (-r_v);

        Eigen::MatrixXd Jp = Eigen::MatrixXd::Zero(3, n_vars);
        Jp.block<3, 3>(0, 3 * static_cast<int>(i)) = -dt * R_i.transpose();
        Jp.block<3, 3>(0, 3 * static_cast<int>(N)) = -pre->jacobian().block<3, 3>(6, 0);
        Jp.block<3, 3>(0, 3 * static_cast<int>(N) + 3) = -pre->jacobian().block<3, 3>(6, 3);
        H += Jp.transpose() * Jp;
        b += Jp.transpose() * (-r_p);
    }

    Eigen::LDLT<Eigen::MatrixXd> ldlt(H + 1e-6 * Eigen::MatrixXd::Identity(n_vars, n_vars));
    if (ldlt.info() != Eigen::Success) {
        return false;
    }
    const Eigen::VectorXd dx = ldlt.solve(b);
    for (std::size_t i = 0; i < N; ++i) {
        velocities_[i] += dx.segment<3>(3 * static_cast<int>(i));
    }
    bias_.acc += dx.segment<3>(3 * static_cast<int>(N));
    bias_.gyro += dx.segment<3>(3 * static_cast<int>(N) + 3);
    return true;
}

bool inertial::apply_to_map(data::map_database* map_db) {
    if (!map_db || state_ != state_t::Succeeded) {
        return false;
    }

    // Scale keyframe translations about the first keyframe camera center.
    const auto all_kfs = map_db->get_all_keyframes();
    if (all_kfs.empty()) {
        return false;
    }
    Vec3_t origin = Vec3_t::Zero();
    if (!keyfrms_.empty()) {
        origin = keyfrms_.front()->get_trans_wc();
    }

    for (const auto& kf : all_kfs) {
        if (!kf || kf->will_be_erased()) {
            continue;
        }
        Mat44_t pose_cw = kf->get_pose_cw();
        Mat33_t R_cw = pose_cw.block<3, 3>(0, 0);
        Vec3_t t_cw = pose_cw.block<3, 1>(0, 3);
        // twc' = origin + s * (twc - origin); pose_cw from twc, R_cw
        Vec3_t twc = -R_cw.transpose() * t_cw;
        twc = origin + scale_ * (twc - origin);
        t_cw = -R_cw * twc;
        pose_cw.block<3, 1>(0, 3) = t_cw;
        kf->set_pose_cw(pose_cw);
        kf->set_imu_bias(bias_);
    }

    // Write velocities for init window keyframes.
    for (std::size_t i = 0; i < keyfrms_.size() && i < velocities_.size(); ++i) {
        keyfrms_[i]->set_velocity(velocities_[i]);
        keyfrms_[i]->set_imu_bias(bias_);
    }

    // Relinearize temporal preintegrators with the solved bias.
    for (std::size_t i = 0; i + 1 < keyfrms_.size(); ++i) {
        if (auto pre = keyfrms_[i + 1]->get_imu_preintegrator()) {
            pre->update_bias(bias_, /*reintegrate_thr=*/0.0);
        }
    }

    for (const auto& lm : map_db->get_all_landmarks()) {
        if (!lm || lm->will_be_erased()) {
            continue;
        }
        lm->set_pos_in_world(origin + scale_ * (lm->get_pos_in_world() - origin));
    }

    AINFO << "inertial init applied to map (scale=" << scale_ << ")";
    return true;
}

}  // namespace initialize
}  // namespace autonomy::localization::atlas

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

#include "autonomy/localization/atlas/imu/preintegrator.hpp"

#include <cmath>
#include <Eigen/Eigenvalues>

namespace autonomy::localization::atlas {
namespace imu {
namespace {

Mat33_t skew(const Vec3_t& v) {
    Mat33_t m;
    m << 0.0, -v(2), v(1),
        v(2), 0.0, -v(0),
        -v(1), v(0), 0.0;
    return m;
}

}  // namespace

preintegrator::preintegrator(const config& cfg, const bias& b)
    : cfg_(cfg), bias_(b) {}

void preintegrator::reset(const bias& b) {
    bias_ = b;
    delta_t_ = 0.0;
    delta_R_ = Mat33_t::Identity();
    delta_v_ = Vec3_t::Zero();
    delta_p_ = Vec3_t::Zero();
    cov_.setZero();
    jac_.setZero();
    valid_ = true;
    has_last_ = false;
    last_meas_ = measurement{};
    // Keep meas_ / t_start_ / t_end_ so reintegrate can rebuild.
}

void preintegrator::set_bias(const bias& b) {
    bias_ = b;
}

Mat33_t preintegrator::exp_so3(const Vec3_t& omega) {
    const double theta = omega.norm();
    const Mat33_t W = skew(omega);
    if (theta < 1e-8) {
        return Mat33_t::Identity() + W;
    }
    const double s = std::sin(theta) / theta;
    const double c = (1.0 - std::cos(theta)) / (theta * theta);
    return Mat33_t::Identity() + s * W + c * W * W;
}

Mat33_t preintegrator::right_jacobian_so3(const Vec3_t& omega) {
    const double theta = omega.norm();
    const Mat33_t W = skew(omega);
    if (theta < 1e-8) {
        return Mat33_t::Identity() - 0.5 * W;
    }
    const double theta2 = theta * theta;
    const double s = std::sin(theta);
    const double c = std::cos(theta);
    return Mat33_t::Identity()
           - (1.0 - c) / theta2 * W
           + (theta - s) / (theta2 * theta) * W * W;
}

void preintegrator::integrate_midpoint(const Vec3_t& a, const Vec3_t& w, double dt) {
    if (dt <= 0.0 || !std::isfinite(dt)) {
        valid_ = false;
        return;
    }

    const Vec3_t acc = a - bias_.acc;
    const Vec3_t gyro = w - bias_.gyro;

    const Mat33_t R = delta_R_;
    const Vec3_t dv = delta_v_;
    const Vec3_t dp = delta_p_;

    // Mid-point rotation update.
    const Vec3_t omega = gyro * dt;
    const Mat33_t dR = exp_so3(omega);
    const Mat33_t Jr = right_jacobian_so3(omega);
    delta_R_ = (R * dR).eval();

    // Position / velocity with rotated acceleration.
    const Vec3_t a_nav = R * acc;
    delta_p_ = dp + dv * dt + 0.5 * a_nav * dt * dt;
    delta_v_ = dv + a_nav * dt;
    delta_t_ += dt;

    // Noise continuous densities -> discrete.
    const double na2 = cfg_.noise_acc * cfg_.noise_acc;
    const double ng2 = cfg_.noise_gyro * cfg_.noise_gyro;
    const double nwa2 = cfg_.random_walk_acc * cfg_.random_walk_acc;
    const double nwg2 = cfg_.random_walk_gyro * cfg_.random_walk_gyro;

    // State order: [dR(3), dv(3), dp(3)]
    Cov9_t A = Cov9_t::Identity();
    A.block<3, 3>(0, 0) = dR.transpose();
    A.block<3, 3>(3, 0) = -R * skew(acc) * dt;
    A.block<3, 3>(6, 0) = -0.5 * R * skew(acc) * dt * dt;
    A.block<3, 3>(6, 3) = Mat33_t::Identity() * dt;

    Cov9_t B = Cov9_t::Zero();
    B.block<3, 3>(0, 0) = Jr * dt;                 // gyro noise -> dR
    B.block<3, 3>(3, 3) = R * dt;                  // acc noise -> dv
    B.block<3, 3>(6, 3) = 0.5 * R * dt * dt;       // acc noise -> dp

    Cov9_t Q = Cov9_t::Zero();
    Q.block<3, 3>(0, 0) = Mat33_t::Identity() * ng2;
    Q.block<3, 3>(3, 3) = Mat33_t::Identity() * na2;
    // indices 6..8 unused in this reduced noise model

    cov_ = A * cov_ * A.transpose() + B * Q * B.transpose();
    // Bias random-walk contribution on velocity/position via process noise on bias is
    // handled in the optimizer; inflate diagonal slightly for stability.
    cov_(3, 3) += nwa2 * dt * dt;
    cov_(4, 4) += nwa2 * dt * dt;
    cov_(5, 5) += nwa2 * dt * dt;
    cov_(0, 0) += nwg2 * dt * dt;
    cov_(1, 1) += nwg2 * dt * dt;
    cov_(2, 2) += nwg2 * dt * dt;

    // Jacobians of [dR,dv,dp] w.r.t. [ba, bg]
    Jac9x6_t J = jac_;
    // d(dR)/d(bg)
    J.block<3, 3>(0, 3) = dR.transpose() * J.block<3, 3>(0, 3) - Jr * dt;
    // d(dv)/d(ba), d(dv)/d(bg)
    J.block<3, 3>(3, 0) = J.block<3, 3>(3, 0) - R * dt;
    J.block<3, 3>(3, 3) = J.block<3, 3>(3, 3) - R * skew(acc) * J.block<3, 3>(0, 3) * dt;
    // d(dp)/d(ba), d(dp)/d(bg)
    J.block<3, 3>(6, 0) = J.block<3, 3>(6, 0) + J.block<3, 3>(3, 0) * dt - 0.5 * R * dt * dt;
    J.block<3, 3>(6, 3) = J.block<3, 3>(6, 3) + J.block<3, 3>(3, 3) * dt
                          - 0.5 * R * skew(acc) * J.block<3, 3>(0, 3) * dt * dt;
    jac_ = J;
}

void preintegrator::integrate(const measurement& m, double dt) {
    integrate_midpoint(m.a, m.w, dt);
    last_meas_ = m;
    has_last_ = true;
}

bool preintegrator::integrate_measurements(const eigen_alloc_vector<measurement>& meas,
                                           double t_start, double t_end) {
    if (meas.empty() || t_end <= t_start) {
        return false;
    }

    meas_ = meas;
    t_start_ = t_start;
    t_end_ = t_end;
    return reintegrate(bias_);
}

bool preintegrator::reintegrate(const bias& b_new) {
    if (meas_.empty() || t_end_ <= t_start_) {
        return false;
    }

    const auto stored = meas_;
    const double t0 = t_start_;
    const double t1 = t_end_;
    reset(b_new);
    meas_ = stored;
    t_start_ = t0;
    t_end_ = t1;

    double t_prev = t_start_;
    measurement prev;
    prev.timestamp = t_start_;
    prev.a = meas_.front().a;
    prev.w = meas_.front().w;

    for (const auto& m : meas_) {
        if (m.timestamp <= t_start_) {
            prev = m;
            t_prev = m.timestamp;
            continue;
        }
        if (m.timestamp > t_end_) {
            break;
        }
        const double dt = m.timestamp - t_prev;
        const Vec3_t a_mid = 0.5 * (prev.a + m.a);
        const Vec3_t w_mid = 0.5 * (prev.w + m.w);
        integrate_midpoint(a_mid, w_mid, dt);
        prev = m;
        t_prev = m.timestamp;
    }

    if (t_prev < t_end_) {
        integrate_midpoint(prev.a, prev.w, t_end_ - t_prev);
    }

    return is_valid();
}

void preintegrator::merge_bias_correction(const bias& b_new) {
    Mat33_t dR;
    Vec3_t dv, dp;
    corrected_deltas(b_new, dR, dv, dp);
    delta_R_ = dR;
    delta_v_ = dv;
    delta_p_ = dp;
    bias_ = b_new;
    jac_.setZero();
}

bool preintegrator::update_bias(const bias& b_new, double reintegrate_thr) {
    const Vec3_t dba = b_new.acc - bias_.acc;
    const Vec3_t dbg = b_new.gyro - bias_.gyro;
    const double db_norm = std::sqrt(dba.squaredNorm() + dbg.squaredNorm());
    if (db_norm < 1e-12) {
        bias_ = b_new;
        return false;
    }
    if (has_measurements() && db_norm > reintegrate_thr) {
        return reintegrate(b_new);
    }
    merge_bias_correction(b_new);
    return true;
}

void preintegrator::corrected_deltas(const bias& b_new, Mat33_t& dR, Vec3_t& dv, Vec3_t& dp) const {
    const Vec3_t dbg = b_new.gyro - bias_.gyro;
    const Vec3_t dba = b_new.acc - bias_.acc;

    // jac_ layout: rows [dR(3), dv(3), dp(3)], cols [ba(3), bg(3)]
    const Vec3_t dphi = jac_.block<3, 3>(0, 3) * dbg;
    dR = delta_R_ * exp_so3(dphi);
    dv = delta_v_ + jac_.block<3, 3>(3, 0) * dba + jac_.block<3, 3>(3, 3) * dbg;
    dp = delta_p_ + jac_.block<3, 3>(6, 0) * dba + jac_.block<3, 3>(6, 3) * dbg;
}

preintegrator::Cov9_t preintegrator::information() const {
    Cov9_t cov = cov_;
    // Regularize for invertibility.
    for (int i = 0; i < 9; ++i) {
        cov(i, i) += 1e-8;
    }
    Eigen::SelfAdjointEigenSolver<Cov9_t> es(cov);
    if (es.info() != Eigen::Success) {
        return Cov9_t::Identity();
    }
    Cov9_t inv = Cov9_t::Zero();
    for (int i = 0; i < 9; ++i) {
        const double ev = std::max(es.eigenvalues()(i), 1e-8);
        inv += (1.0 / ev) * es.eigenvectors().col(i) * es.eigenvectors().col(i).transpose();
    }
    return 0.5 * (inv + inv.transpose());
}

void preintegrator::predict(const Mat33_t& R_wb, const Vec3_t& t_wb, const Vec3_t& v_w,
                            const Vec3_t& gravity,
                            Mat33_t& R_wb_pred, Vec3_t& t_wb_pred, Vec3_t& v_w_pred) const {
    const double dt = delta_t_;
    R_wb_pred = R_wb * delta_R_;
    v_w_pred = v_w + gravity * dt + R_wb * delta_v_;
    t_wb_pred = t_wb + v_w * dt + 0.5 * gravity * dt * dt + R_wb * delta_p_;
}

bool preintegrator::predict_camera_pose(const Mat44_t& pose_cw_prev, const Vec3_t& v_w,
                                        const Vec3_t& gravity, Mat44_t& pose_cw_pred,
                                        Vec3_t& v_w_pred) const {
    if (!is_valid()) {
        return false;
    }

    const Mat44_t T_c_b = cfg_.T_c_b();
    const Mat44_t T_b_c = T_c_b.inverse();

    // pose_cw: world -> camera. Convert to body (IMU) world pose.
    const Mat44_t pose_bw_prev = T_b_c * pose_cw_prev;  // world -> body
    Mat33_t R_wb = pose_bw_prev.block<3, 3>(0, 0).transpose();
    Vec3_t t_wb = -R_wb * pose_bw_prev.block<3, 1>(0, 3);

    Mat33_t R_wb_pred;
    Vec3_t t_wb_pred;
    predict(R_wb, t_wb, v_w, gravity, R_wb_pred, t_wb_pred, v_w_pred);

    Mat44_t pose_wb_pred = Mat44_t::Identity();
    pose_wb_pred.block<3, 3>(0, 0) = R_wb_pred;
    pose_wb_pred.block<3, 1>(0, 3) = t_wb_pred;
    const Mat44_t pose_bw_pred = pose_wb_pred.inverse();
    pose_cw_pred = T_c_b * pose_bw_pred;
    return true;
}

std::shared_ptr<preintegrator> preintegrator::clone() const {
    return std::make_shared<preintegrator>(*this);
}

}  // namespace imu
}  // namespace autonomy::localization::atlas

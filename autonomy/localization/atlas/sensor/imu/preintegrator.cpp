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
 *
 * Preintegration adapted from ORB-SLAM3 ImuTypes.cc (Forster mid-point form).
 */

/**
 * @file
 * @brief imu::Preintegrator / IntegratedRotation implementation (Forster midpoint).
 */

#include "autonomy/localization/atlas/sensor/imu/types.hpp"

#include <cmath>

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace imu {
namespace {

constexpr double kEps = 1e-4;

Mat33 Skew(const Vec3& v) {
    Mat33 m;
    m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return m;
}

Mat33 ExpSO3(const Vec3& w) {
    const double d2 = w.squaredNorm();
    const double d = std::sqrt(d2);
    const Mat33 W = Skew(w);
    if (d < kEps) {
        return Mat33::Identity() + W;
    }
    return Mat33::Identity() + W * std::sin(d) / d +
           W * W * (1.0 - std::cos(d)) / d2;
}

}  // namespace

Mat33 Preintegrator::NormalizeRotation(const Mat33& R) {
    Eigen::JacobiSVD<Mat33> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU() * svd.matrixV().transpose();
}

IntegratedRotation::IntegratedRotation(const Vec3& angular_velocity,
                                       const Bias& bias, double dt) {
    delta_t = dt;
    const Vec3 x = (angular_velocity - bias.gyroscope) * dt;
    const double d2 = x.squaredNorm();
    const double d = std::sqrt(d2);
    const Mat33 W = Skew(x);
    if (d < kEps) {
        delta_R = Mat33::Identity() + W;
        right_jacobian = Mat33::Identity();
    } else {
        delta_R = Mat33::Identity() + W * std::sin(d) / d +
                  W * W * (1.0 - std::cos(d)) / d2;
        right_jacobian = Mat33::Identity() - W * (1.0 - std::cos(d)) / d2 +
                         W * W * (d - std::sin(d)) / (d2 * d);
    }
}

Preintegrator::Preintegrator(const Bias& b, const Calib& calib)
    : Nga(calib.covariance), NgaWalk(calib.covariance_walk) {
    Initialize(b);
}

Preintegrator::Preintegrator(const Preintegrator& other) {
    std::lock_guard<std::mutex> lock(other.mutex_);
    delta_t = other.delta_t;
    covariance = other.covariance;
    information = other.information;
    Nga = other.Nga;
    NgaWalk = other.NgaWalk;
    bias = other.bias;
    dR = other.dR;
    dV = other.dV;
    dP = other.dP;
    JRg = other.JRg;
    JVg = other.JVg;
    JVa = other.JVa;
    JPg = other.JPg;
    JPa = other.JPa;
    avg_A = other.avg_A;
    avg_W = other.avg_W;
    bias_updated_ = other.bias_updated_;
    db_ = other.db_;
    measurements_ = other.measurements_;
}

void Preintegrator::Initialize(const Bias& b) {
    dR.setIdentity();
    dV.setZero();
    dP.setZero();
    JRg.setZero();
    JVg.setZero();
    JVa.setZero();
    JPg.setZero();
    JPa.setZero();
    covariance.setZero();
    information.setZero();
    db_.setZero();
    bias = b;
    bias_updated_ = b;
    avg_A.setZero();
    avg_W.setZero();
    delta_t = 0.0;
    measurements_.clear();
}

void Preintegrator::IntegrateNewMeasurement(const Vec3& acceleration,
                                            const Vec3& angular_velocity,
                                            double dt) {
    measurements_.push_back({acceleration, angular_velocity, dt});

    Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Identity();
    Eigen::Matrix<double, 9, 6> B = Eigen::Matrix<double, 9, 6>::Zero();

    const Vec3 acc = acceleration - bias.accelerometer;
    const Vec3 acc_w = angular_velocity - bias.gyroscope;

    avg_A = (delta_t * avg_A + dR * acc * dt) / (delta_t + dt);
    avg_W = (delta_t * avg_W + acc_w * dt) / (delta_t + dt);

    dP = dP + dV * dt + 0.5 * dR * acc * dt * dt;
    dV = dV + dR * acc * dt;

    const Mat33 Wacc = Skew(acc);
    A.block<3, 3>(3, 0) = -dR * dt * Wacc;
    A.block<3, 3>(6, 0) = -0.5 * dR * dt * dt * Wacc;
    A.block<3, 3>(6, 3) = Mat33::Identity() * dt;
    B.block<3, 3>(3, 3) = dR * dt;
    B.block<3, 3>(6, 3) = 0.5 * dR * dt * dt;

    JPa = JPa + JVa * dt - 0.5 * dR * dt * dt;
    JPg = JPg + JVg * dt - 0.5 * dR * dt * dt * Wacc * JRg;
    JVa = JVa - dR * dt;
    JVg = JVg - dR * dt * Wacc * JRg;

    IntegratedRotation dRi(angular_velocity, bias, dt);
    dR = NormalizeRotation(dR * dRi.delta_R);

    A.block<3, 3>(0, 0) = dRi.delta_R.transpose();
    B.block<3, 3>(0, 0) = dRi.right_jacobian * dt;

    covariance.block<9, 9>(0, 0) =
        A * covariance.block<9, 9>(0, 0) * A.transpose() +
        B * Nga * B.transpose();
    covariance.block<6, 6>(9, 9) += NgaWalk;

    JRg = dRi.delta_R.transpose() * JRg - dRi.right_jacobian * dt;
    delta_t += dt;
}

void Preintegrator::Reintegrate() {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto aux = measurements_;
    const auto nga = Nga;
    const auto nga_walk = NgaWalk;
    Initialize(bias_updated_);
    Nga = nga;
    NgaWalk = nga_walk;
    for (const auto& m : aux) {
        IntegrateNewMeasurement(m.a, m.w, m.t);
    }
    RefreshInformation();
}

void Preintegrator::MergePrevious(const Preintegrator& previous) {
    if (&previous == this) {
        return;
    }
    std::lock_guard<std::mutex> lock1(mutex_);
    std::lock_guard<std::mutex> lock2(previous.mutex_);
    // Preserve noise models from *this* (ORB-SLAM3 MergePrevious).
    const auto nga = Nga;
    const auto nga_walk = NgaWalk;
    Bias bav = bias_updated_;
    const auto aux1 = previous.measurements_;
    const auto aux2 = measurements_;
    Initialize(bav);
    Nga = nga;
    NgaWalk = nga_walk;
    for (const auto& m : aux1) {
        IntegrateNewMeasurement(m.a, m.w, m.t);
    }
    for (const auto& m : aux2) {
        IntegrateNewMeasurement(m.a, m.w, m.t);
    }
    RefreshInformation();
}

void Preintegrator::RefreshInformation() {
    // Symmetric inverse of covariance → information (full 15×15).
    Eigen::Matrix<double, 15, 15> C =
        0.5 * (covariance + covariance.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> es(C);
    Eigen::Matrix<double, 15, 1> eigs = es.eigenvalues();
    for (int i = 0; i < 15; ++i) {
        eigs[i] = (eigs[i] > 1e-12) ? (1.0 / eigs[i]) : 0.0;
    }
    information =
        es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
}

void Preintegrator::SetNewBias(const Bias& bu) {
    std::lock_guard<std::mutex> lock(mutex_);
    bias_updated_ = bu;
    db_.head<3>() = bu.gyroscope - bias.gyroscope;
    db_.tail<3>() = bu.accelerometer - bias.accelerometer;
}

Mat33 Preintegrator::GetDeltaRotation(const Bias& b) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const Vec3 dbg = b.gyroscope - bias.gyroscope;
    return NormalizeRotation(dR * ExpSO3(JRg * dbg));
}

Vec3 Preintegrator::GetDeltaVelocity(const Bias& b) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const Vec3 dbg = b.gyroscope - bias.gyroscope;
    const Vec3 dba = b.accelerometer - bias.accelerometer;
    return dV + JVg * dbg + JVa * dba;
}

Vec3 Preintegrator::GetDeltaPosition(const Bias& b) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const Vec3 dbg = b.gyroscope - bias.gyroscope;
    const Vec3 dba = b.accelerometer - bias.accelerometer;
    return dP + JPg * dbg + JPa * dba;
}

Mat33 Preintegrator::GetUpdatedDeltaRotation() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return NormalizeRotation(dR * ExpSO3(JRg * db_.head<3>()));
}

Vec3 Preintegrator::GetUpdatedDeltaVelocity() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dV + JVg * db_.head<3>() + JVa * db_.tail<3>();
}

Vec3 Preintegrator::GetUpdatedDeltaPosition() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dP + JPg * db_.head<3>() + JPa * db_.tail<3>();
}

Mat33 Preintegrator::GetOriginalDeltaRotation() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dR;
}

Vec3 Preintegrator::GetOriginalDeltaVelocity() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dV;
}

Vec3 Preintegrator::GetOriginalDeltaPosition() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dP;
}

Bias Preintegrator::GetOriginalBias() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return bias;
}

Bias Preintegrator::GetUpdatedBias() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return bias_updated_;
}

Eigen::Matrix<double, 6, 1> Preintegrator::GetDeltaBias() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return db_;
}

Preintegrator::State Preintegrator::ExportState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    State s;
    s.delta_t = delta_t;
    s.covariance = covariance;
    s.information = information;
    s.nga_diag = Nga.diagonal();
    s.nga_walk_diag = NgaWalk.diagonal();
    s.bias = bias;
    s.bias_updated = bias_updated_;
    s.dR = dR;
    s.dV = dV;
    s.dP = dP;
    s.JRg = JRg;
    s.JVg = JVg;
    s.JVa = JVa;
    s.JPg = JPg;
    s.JPa = JPa;
    s.avg_A = avg_A;
    s.avg_W = avg_W;
    s.db = db_;
    s.measurements.reserve(measurements_.size());
    for (const auto& m : measurements_) {
        Eigen::Matrix<double, 7, 1> row;
        row << m.a.x(), m.a.y(), m.a.z(), m.w.x(), m.w.y(), m.w.z(), m.t;
        s.measurements.push_back(row);
    }
    return s;
}

void Preintegrator::ImportState(const State& state) {
    std::lock_guard<std::mutex> lock(mutex_);
    delta_t = state.delta_t;
    covariance = state.covariance;
    information = state.information;
    Nga.diagonal() = state.nga_diag;
    NgaWalk.diagonal() = state.nga_walk_diag;
    bias = state.bias;
    bias_updated_ = state.bias_updated;
    dR = state.dR;
    dV = state.dV;
    dP = state.dP;
    JRg = state.JRg;
    JVg = state.JVg;
    JVa = state.JVa;
    JPg = state.JPg;
    JPa = state.JPa;
    avg_A = state.avg_A;
    avg_W = state.avg_W;
    db_ = state.db;
    measurements_.clear();
    measurements_.reserve(state.measurements.size());
    for (const auto& row : state.measurements) {
        Integrable m;
        m.a = Vec3(row(0), row(1), row(2));
        m.w = Vec3(row(3), row(4), row(5));
        m.t = row(6);
        measurements_.push_back(m);
    }
}

}  // namespace imu
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

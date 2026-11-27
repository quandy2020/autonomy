// Copyright 2026 The Openbot Authors
#include "autonomy/localization/fusion/planar_ekf.hpp"

#include <cmath>
#include <stdexcept>
#include <Eigen/Cholesky>

namespace autonomy::localization::fusion {
namespace {
double Wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
}

PlanarEkf::PlanarEkf(Options options) : options_(options) {
    for (double v : {options.acceleration_noise, options.angular_acceleration_noise,
                     options.wheel_timeout})
        if (!std::isfinite(v) || v <= 0.) throw std::invalid_argument("Invalid planar EKF noise/timeout");
}

Eigen::Matrix<double, 6, 1> PlanarEkf::state() const {
    if (!initialized_) throw std::logic_error("EKF has no wheel measurements");
    Eigen::MatrixXd value;
    filter_.getX(value);
    return value;
}

Eigen::Matrix<double, 6, 6> PlanarEkf::covariance() const {
    if (!initialized_) throw std::logic_error("EKF has no wheel measurements");
    Eigen::MatrixXd value;
    filter_.getP(value);
    return value;
}

void PlanarEkf::Normalize() {
    auto x = state();
    const auto p = covariance();
    x(2) = Wrap(x(2));
    filter_.init(x, (p + p.transpose()).eval() * .5);
}

bool PlanarEkf::Predict(double time) {
    if (!initialized_ || !std::isfinite(time) || time < time_ ||
        time - wheel_time_ > options_.wheel_timeout) return false;
    const double dt = time - time_;
    if (dt == 0.) return true;
    auto x = state();
    const double heading = x(2) + .5 * dt * x(5);
    const double c = std::cos(heading), s = std::sin(heading);
    const double vx = c * x(3) - s * x(4), vy = s * x(3) + c * x(4);
    Eigen::Matrix<double, 6, 6> f = Eigen::Matrix<double, 6, 6>::Identity();
    f(0, 2) = -dt * vy; f(1, 2) = dt * vx;
    f(0, 3) = dt * c; f(0, 4) = -dt * s;
    f(1, 3) = dt * s; f(1, 4) = dt * c;
    f(0, 5) = -.5 * dt * dt * vy; f(1, 5) = .5 * dt * dt * vx;
    f(2, 5) = dt;
    Eigen::Matrix<double, 6, 3> g = Eigen::Matrix<double, 6, 3>::Zero();
    g(0, 0) = .5 * dt * c; g(0, 1) = -.5 * dt * s;
    g(1, 0) = .5 * dt * s; g(1, 1) = .5 * dt * c;
    g(2, 2) = .5 * dt;
    g.bottomRows<3>().setIdentity();
    Eigen::Vector3d noise;
    noise << std::pow(options_.acceleration_noise, 2), std::pow(options_.acceleration_noise, 2),
             std::pow(options_.angular_acceleration_noise, 2);
    const Eigen::Matrix<double, 6, 6> q = g * noise.asDiagonal() * g.transpose() * dt;
    x(0) += dt * vx; x(1) += dt * vy; x(2) = Wrap(x(2) + dt * x(5));
    if (!x.allFinite() || !q.allFinite() || !f.allFinite()) return false;
    if (!filter_.predict(x, f, q)) return false;
    time_ = time;
    Normalize();
    return true;
}

bool PlanarEkf::AddWheel(double time, const Eigen::Vector3d& velocity,
                         const Eigen::Matrix3d& covariance) {
    if (!std::isfinite(time) || time < 0. || !velocity.allFinite() || !covariance.allFinite() ||
        !covariance.isApprox(covariance.transpose(), 1e-9) ||
        Eigen::LLT<Eigen::Matrix3d>(covariance).info() != Eigen::Success ||
        (initialized_ && (time < time_ || time <= wheel_time_))) return false;
    if (!initialized_ || time - wheel_time_ > options_.wheel_timeout) {
        Eigen::Matrix<double, 6, 1> x = Eigen::Matrix<double, 6, 1>::Zero();
        Eigen::Matrix<double, 6, 6> p = Eigen::Matrix<double, 6, 6>::Identity() * 1e-6;
        if (initialized_) {
            x = state();
            // Motion during a sensor outage is unknown; preserve continuity and increase uncertainty.
            p.topLeftCorner<3, 3>() = this->covariance().topLeftCorner<3, 3>() +
                Eigen::Matrix3d::Identity() * (time - wheel_time_);
        }
        x.tail<3>() = velocity;
        p.bottomRightCorner<3, 3>() = covariance;
        filter_.init(x, p);
        initialized_ = true;
        time_ = wheel_time_ = time;
        return true;
    }
    // Predict with the previous motion before incorporating this new measurement.
    if (!Predict(time)) return false;
    Eigen::Matrix<double, 3, 6> h = Eigen::Matrix<double, 3, 6>::Zero();
    h.rightCols<3>().setIdentity();
    if (!filter_.update(velocity, h, covariance)) return false;
    wheel_time_ = time;
    Normalize();
    return true;
}

bool PlanarEkf::AddGyro(double time, double yaw_rate, double variance) {
    if (!std::isfinite(yaw_rate) || !std::isfinite(variance) || variance <= 0. ||
        time <= gyro_time_ || !Predict(time)) return false;
    Eigen::Matrix<double, 1, 6> h = Eigen::Matrix<double, 1, 6>::Zero();
    h(5) = 1.;
    Eigen::Matrix<double, 1, 1> y, r;
    y(0) = yaw_rate; r(0) = variance;
    if (!filter_.update(y, h, r)) return false;
    gyro_time_ = time;
    Normalize();
    return true;
}

}  // namespace autonomy::localization::fusion

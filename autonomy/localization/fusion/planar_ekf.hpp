// Copyright 2026 The Openbot Authors
#pragma once

#include <Eigen/Core>

#include "autonomy/common/kalman_filter/kalman_filter.hpp"

namespace autonomy::localization::fusion {

// Body-frame velocities; pose is in the continuous, locally fixed odom frame.
class PlanarEkf {
public:
    struct Options {
        double acceleration_noise = 0.5;
        double angular_acceleration_noise = 0.5;
        double wheel_timeout = 0.5;
    };
    explicit PlanarEkf(Options options);
    bool AddWheel(double time, const Eigen::Vector3d& velocity,
                  const Eigen::Matrix3d& covariance);
    bool AddGyro(double time, double yaw_rate, double variance);
    bool initialized() const { return initialized_; }
    double time() const { return time_; }
    Eigen::Matrix<double, 6, 1> state() const;
    Eigen::Matrix<double, 6, 6> covariance() const;

private:
    bool Predict(double time);
    void Normalize();
    Options options_;
    common::kalman_filter::KalmanFilter filter_;
    bool initialized_ = false;
    double time_ = 0.;
    double wheel_time_ = 0.;
    double gyro_time_ = -1.;
};

}  // namespace autonomy::localization::fusion

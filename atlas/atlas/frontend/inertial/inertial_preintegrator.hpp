/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file inertial_preintegrator.hpp
 * @brief Voxel-SLAM IMU preintegration (InertialPreintegrator) without ROS
 * messages.
 */

#ifndef ATLAS_FRONTEND_INERTIAL_INERTIAL_PREINTEGRATOR_HPP_
#define ATLAS_FRONTEND_INERTIAL_INERTIAL_PREINTEGRATOR_HPP_

#include <Eigen/Core>
#include <deque>
#include <vector>

#include "frontend/inertial/inertial_state.hpp"
#include "sensor/imu_data.hpp"

namespace atlas {
namespace frontend {
namespace inertial {

/** @brief Per-trajectory noise and unit conversion for IMU preintegration. */
struct InertialPreintegrationOptions {
  double gravity_scale = 1.0;
  Eigen::Matrix<double, 6, 6> measurement_noise =
      Eigen::Matrix<double, 6, 6>::Identity() * 0.1;
  Eigen::Matrix<double, 6, 6> bias_random_walk_noise =
      Eigen::Matrix<double, 6, 6>::Identity() * 1e-4;
};

/** @brief Builds diagonal IMU noise covariances from per-axis variances. */
InertialPreintegrationOptions MakeInertialPreintegrationOptions(
    double gravity_scale, const Eigen::Vector3d& gyroscope_noise,
    const Eigen::Vector3d& accelerometer_noise,
    const Eigen::Vector3d& gyroscope_bias_random_walk,
    const Eigen::Vector3d& accelerometer_bias_random_walk);

/**
 * @brief Mid-point IMU preintegration between keyframes for LI-BA.
 */
class InertialPreintegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit InertialPreintegrator(
      const Eigen::Vector3d& bg1 = Eigen::Vector3d::Zero(),
      const Eigen::Vector3d& ba1 = Eigen::Vector3d::Zero(),
      const InertialPreintegrationOptions& options =
          InertialPreintegrationOptions());

  void PushImu(const std::deque<sensor::ImuData>& imus);
  void AddImu(Eigen::Vector3d cur_gyr, Eigen::Vector3d cur_acc, double dt);

  double Evaluate(InertialState& st1, InertialState& st2, Eigen::MatrixXd& jtj,
                  Eigen::VectorXd& gg, bool jac_enable);
  double EvaluateGravity(InertialState& st1, InertialState& st2,
                         Eigen::MatrixXd& jtj, Eigen::VectorXd& gg,
                         bool jac_enable);

  void ApplyDelta(const Eigen::Matrix<double, kInertialStateDimension, 1>& dxi);
  void Merge(const InertialPreintegrator& imu2);

  Eigen::Matrix3d R_delta = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_delta = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_delta = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();

  Eigen::Matrix3d R_bg = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d p_bg = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d p_ba = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d v_bg = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d v_ba = Eigen::Matrix3d::Zero();

  double dtime = 0.0;
  Eigen::Vector3d dbg = Eigen::Vector3d::Zero();
  Eigen::Vector3d dba = Eigen::Vector3d::Zero();
  Eigen::Vector3d dbg_buf = Eigen::Vector3d::Zero();
  Eigen::Vector3d dba_buf = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, kInertialStateDimension, kInertialStateDimension> cov =
      Eigen::Matrix<double, kInertialStateDimension,
                    kInertialStateDimension>::Zero();

  std::deque<sensor::ImuData> imus;

 private:
  InertialPreintegrationOptions options_;
};

}  // namespace inertial
}  // namespace frontend
}  // namespace atlas

#endif  // ATLAS_FRONTEND_INERTIAL_INERTIAL_PREINTEGRATOR_HPP_

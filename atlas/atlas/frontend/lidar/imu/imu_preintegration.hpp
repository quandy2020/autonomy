/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file imu_preintegration.hpp
 * @brief Voxel-SLAM IMU preintegration (ImuPreintegration) without ROS
 * messages.
 */

#ifndef ATLAS_VXSLAM_IMU_PREINTEGRATION_HPP_
#define ATLAS_VXSLAM_IMU_PREINTEGRATION_HPP_

#include <Eigen/Core>
#include <deque>
#include <vector>

#include "frontend/lidar/imu/imu_state.hpp"
#include "sensor/imu_types.hpp"

namespace atlas {
namespace vxslam {

/** @brief Global gravity scale for preintegration (Voxel-SLAM imupre_scale). */
extern double g_scale_gravity;
/** @brief Measurement noise 6×6 (gyro, acc). */
extern Eigen::Matrix<double, 6, 6> g_noise_meas;
/** @brief Bias random-walk noise 6×6. */
extern Eigen::Matrix<double, 6, 6> g_noise_walk;

/**
 * @brief Mid-point IMU preintegration between keyframes for LI-BA.
 */
class ImuPreintegration {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ImuPreintegration(
      const Eigen::Vector3d& bg1 = Eigen::Vector3d::Zero(),
      const Eigen::Vector3d& ba1 = Eigen::Vector3d::Zero());

  void PushImu(const std::deque<sensor::ImuData>& imus);
  void AddImu(Eigen::Vector3d cur_gyr, Eigen::Vector3d cur_acc, double dt);

  double Evaluate(ImuState& st1, ImuState& st2, Eigen::MatrixXd& jtj,
                  Eigen::VectorXd& gg, bool jac_enable);
  double EvaluateGravity(ImuState& st1, ImuState& st2, Eigen::MatrixXd& jtj,
                         Eigen::VectorXd& gg, bool jac_enable);

  void ApplyDelta(const Eigen::Matrix<double, kStateDim, 1>& dxi);
  void Merge(const ImuPreintegration& imu2);

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
  Eigen::Matrix<double, kStateDim, kStateDim> cov =
      Eigen::Matrix<double, kStateDim, kStateDim>::Zero();

  std::deque<sensor::ImuData> imus;
};

}  // namespace vxslam
}  // namespace atlas

#endif  // ATLAS_VXSLAM_IMU_PREINTEGRATION_HPP_

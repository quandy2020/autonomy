/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file imu_ekf.hpp
 * @brief Voxel-SLAM IMUEKF: init, predict, motion undistortion (no ROS).
 */

#ifndef ARGUS_VXSLAM_IMU_EKF_HPP_
#define ARGUS_VXSLAM_IMU_EKF_HPP_

#include <Eigen/Core>
#include <deque>
#include <vector>

#include "frontend/lidar/imu/imu_state.hpp"
#include "sensor/imu_types.hpp"
#include "sensor/point_types.hpp"

namespace argus {
namespace vxslam {

/**
 * @brief Front-end IMU EKF for Voxel-SLAM pipeline.
 *
 * @return Process(): 0 while initializing, 1 after motion_blur update.
 */
class ImuEkf {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool init_flag = false;
  double pcl_beg_time = 0.0;
  double pcl_end_time = 0.0;
  double last_pcl_end_time = 0.0;
  int init_num = 0;
  int min_init_num = 30;
  Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_gyr = Eigen::Vector3d::Zero();
  sensor::ImuData last_imu;
  Eigen::Vector3d angvel_last = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_s_last = Eigen::Vector3d::Zero();

  Eigen::Vector3d cov_acc = Eigen::Vector3d::Ones() * 0.1;
  Eigen::Vector3d cov_gyr = Eigen::Vector3d::Ones() * 0.1;
  Eigen::Vector3d cov_bias_gyr = Eigen::Vector3d::Ones() * 0.0001;
  Eigen::Vector3d cov_bias_acc = Eigen::Vector3d::Ones() * 0.0001;

  Eigen::Matrix3d lid_rot_to_imu = Eigen::Matrix3d::Identity();
  Eigen::Vector3d lid_offset_to_imu = Eigen::Vector3d::Zero();
  double scale_gravity = 1.0;
  int point_notime = 0;
  /** @brief When true, treat Livox-scale IMU (norm≈1) as g-units. */
  bool livox_imu_scale = false;

  std::vector<ImuState> imu_poses;

  void MotionBlur(ImuState* xc, sensor::PointCloudXYZI* pcl_in,
                  std::deque<sensor::ImuData>* imus);
  void ImuInit(const std::deque<sensor::ImuData>& imus);

  /**
   * @brief Init or undistort one LiDAR package.
   * @return 0 initializing, 1 ready.
   */
  int Process(ImuState* x_curr, sensor::PointCloudXYZI* pcl_in,
              std::deque<sensor::ImuData>* imus);
};

}  // namespace vxslam
}  // namespace argus

#endif  // ARGUS_VXSLAM_IMU_EKF_HPP_

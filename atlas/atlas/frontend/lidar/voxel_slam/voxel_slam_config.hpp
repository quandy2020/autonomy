/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file odometry_config.hpp
 * @brief Configuration for Voxel-SLAM pipeline inside Argus.
 */

#ifndef ATLAS_VXSLAM_ODOMETRY_CONFIG_HPP_
#define ATLAS_VXSLAM_ODOMETRY_CONFIG_HPP_

#include <Eigen/Core>
#include <string>
#include <vector>

#include "backend/loop_closure/lidar/triangle_descriptor/triangle_types.hpp"
#include "frontend/lidar/odometry/lidar_vendor.hpp"

namespace atlas {
namespace vxslam {

struct OdometryConfig {
  /** @brief Pipeline id (Voxel-SLAM only). */
  std::string pipeline = "voxel_slam";

  LidarVendorOptions lidar;
  Eigen::Matrix3d extrinsic_R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d extrinsic_t = Eigen::Vector3d::Zero();

  double down_size = 0.5;
  double voxel_size = 1.0;
  double min_eigen_value = 0.01;
  std::vector<double> plane_eigenvalue_thresholds = {1.0, 1.0, 1.0};
  int max_layer = 2;
  int win_size = 10;
  double imu_coef = 1e-4;
  int worker_thread_count = 4;
  int degrade_bound = 50;
  bool point_notime = false;
  bool livox_imu_scale = false;

  double dept_err = 0.05;
  double beam_err = 0.2;
  Eigen::Vector3d cov_gyr = Eigen::Vector3d::Ones() * 0.1;
  Eigen::Vector3d cov_acc = Eigen::Vector3d::Ones() * 0.1;

  bool enable_local_ba = true;
  bool enable_gravity_ba = false;
  bool enable_loop = true;
  bool enable_triangle = true;
  bool enable_gba = true;
  /** @brief Min eigenvalue of NormalIcp Gram matrix (Voxel-SLAM
   * Loop/icp_eigval). */
  double icp_eigval = 0.01;
  /** @brief BTC score gate for current session (Loop/jud_default). */
  double jud_default = 0.45;
  /** @brief Max drift/jour for accepting same-session loop. */
  double ratio_drift = 0.05;
  /** @brief Keyframes between same-session ISAM2 triggers (Loop/curr_halt). */
  int curr_halt = 10;
  /** @brief Keyframes between cross-session ISAM2 triggers (Loop/prev_halt). */
  int prev_halt = 30;
  int gba_local_win = 10;
  int gba_merge_size = 5;

  std::string save_path;
  std::string bagname = "session0";
  /** @brief Comma list "name:jud, ..." for previous maps. */
  std::string previous_map;

  loop_closure::triangle::TriangleConfig triangle;
};

}  // namespace vxslam
}  // namespace atlas

#endif  // ATLAS_VXSLAM_ODOMETRY_CONFIG_HPP_

/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file trajectory_builder_options.hpp
 * @brief Configuration for Voxel-SLAM pipeline inside Argus.
 */

#ifndef ATLAS_FRONTEND_TRAJECTORY_BUILDER_OPTIONS_HPP_
#define ATLAS_FRONTEND_TRAJECTORY_BUILDER_OPTIONS_HPP_

#include <Eigen/Core>
#include <string>
#include <vector>

#include "backend/loop_closure/triangle/triangle_descriptor.hpp"
#include "frontend/odometry/lidar_vendor.hpp"

namespace atlas {
namespace frontend {

namespace loop_closure = ::atlas::backend::loop_closure;

using ::atlas::frontend::odometry::LidarVendorOptions;

struct TrajectoryBuilderOptions {
  /** @brief Pipeline id (Voxel-SLAM only). */
  std::string pipeline = "voxel_slam";

  LidarVendorOptions lidar;
  Eigen::Matrix3d lidar_to_imu_rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d lidar_to_imu_translation = Eigen::Vector3d::Zero();

  double point_downsampling_size_meters = 0.5;
  double voxel_size = 1.0;
  double minimum_plane_eigenvalue = 0.01;
  std::vector<double> plane_eigenvalue_thresholds = {1.0, 1.0, 1.0};
  int maximum_voxel_layer = 2;
  int local_window_size = 10;
  double inertial_factor_coefficient = 1e-4;
  int worker_thread_count = 4;
  int degradation_count_limit = 10;
  double minimum_initialization_observability_eigenvalue = 15.0;
  double minimum_odometry_normal_eigenvalue = 14.0;
  bool point_time_unavailable = false;
  bool livox_imu_scale_enabled = false;

  double range_noise_meters = 0.05;
  double beam_angle_noise_degrees = 0.2;
  Eigen::Vector3d odometry_gyroscope_covariance =
      Eigen::Vector3d::Ones() * 0.1;
  Eigen::Vector3d odometry_accelerometer_covariance =
      Eigen::Vector3d::Ones() * 0.1;
  Eigen::Vector3d local_mapping_gyroscope_covariance =
      Eigen::Vector3d::Ones() * 0.1;
  Eigen::Vector3d local_mapping_accelerometer_covariance =
      Eigen::Vector3d::Ones() * 0.1;
  Eigen::Vector3d gyroscope_bias_random_walk = Eigen::Vector3d::Ones() * 1e-4;
  Eigen::Vector3d accelerometer_bias_random_walk =
      Eigen::Vector3d::Ones() * 1e-4;

  bool local_bundle_adjustment_enabled = true;
  bool gravity_bundle_adjustment_enabled = false;
  bool loop_closure_enabled = true;
  bool triangle_descriptors_enabled = true;
  bool global_bundle_adjustment_enabled = true;
  /** @brief Min eigenvalue of NormalIcp Gram matrix (Voxel-SLAM
   * Loop/loop_icp_minimum_eigenvalue). */
  double loop_icp_minimum_eigenvalue = 0.01;
  /** @brief BTC score gate for current session
   * (Loop/descriptor_score_threshold). */
  double descriptor_score_threshold = 0.45;
  /** @brief Max drift/jour for accepting same-session loop. */
  double maximum_drift_ratio = 0.05;
  /** @brief Keyframes between same-session ISAM2 triggers
   * (Loop/same_session_update_interval). */
  int same_session_update_interval = 10;
  /** @brief Keyframes between cross-session ISAM2 triggers
   * (Loop/historical_session_update_interval). */
  int historical_session_update_interval = 30;
  int global_mapping_local_window_size = 10;
  int global_mapping_submap_merge_size = 5;
  double global_mapping_voxel_size = 1.0;
  double global_mapping_minimum_plane_eigenvalue = 0.01;
  std::vector<double> global_mapping_eigenvalue_thresholds = {2.0, 2.0, 2.0,
                                                              2.0};
  int global_mapping_maximum_iteration_count = 3;

  std::string storage_directory;
  std::string session_name = "session0";
  /** @brief Comma list "name:jud, ..." for previous maps. */
  std::string previous_sessions;

  loop_closure::LoopClosureOptions triangle;
};

}  // namespace frontend
}  // namespace atlas

#endif  // ATLAS_FRONTEND_TRAJECTORY_BUILDER_OPTIONS_HPP_

/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_VXSLAM_SESSION_IO_HPP_
#define ATLAS_VXSLAM_SESSION_IO_HPP_

#include <string>
#include <vector>

#include "frontend/lidar/imu/imu_state.hpp"
#include "frontend/lidar/odometry/point_var.hpp"
#include "sensor/point_types.hpp"

namespace atlas {
namespace vxslam {

struct SessionPoseEntry {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuState x;
  Eigen::Matrix<double, 6, 1> v6 = Eigen::Matrix<double, 6, 1>::Zero();
  double jud = 0.5;
};

struct PreviousMapSpec {
  std::string name;
  double jud = 0.5;
};

std::vector<PreviousMapSpec> ParsePreviousMapList(const std::string& text);

bool SaveTrajectoryTum(const std::string& path,
                       const std::vector<ImuState>& poses);
bool LoadTrajectoryTum(const std::string& path, std::vector<ImuState>* poses);

bool SaveMapXyz(const std::string& path, const sensor::PointCloudXYZI& cloud);

/**
 * @brief Voxel-SLAM pose file: t p q v bg ba g v6 (save_pose).
 */
bool SaveSessionPoses(const std::string& path,
                      const std::vector<SessionPoseEntry>& poses);
bool LoadSessionPoses(const std::string& path,
                      std::vector<SessionPoseEntry>* poses);

/**
 * @brief Body-frame cloud dump as binary XYZI (replaces PCL PCD).
 *        Layout: uint64 n, then n * (float x,y,z,intensity).
 */
bool SaveCloudBin(const std::string& path, const PointVarList& pvec);
bool LoadCloudBin(const std::string& path, PointVarList* pvec);
bool SaveCloudBin(const std::string& path, const sensor::PointCloudXYZI& cloud);
bool LoadCloudBin(const std::string& path, sensor::PointCloudXYZI* cloud);

}  // namespace vxslam
}  // namespace atlas

#endif

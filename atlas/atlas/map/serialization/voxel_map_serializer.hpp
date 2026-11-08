/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_MAP_SESSION_IO_HPP_
#define ATLAS_MAP_SESSION_IO_HPP_

#include <string>
#include <vector>

#include "frontend/inertial/inertial_state.hpp"
#include "map/point_observation.hpp"
#include "sensor/point_types.hpp"

namespace atlas {
namespace map {
namespace serialization {

using ::atlas::map::PointObservationList;

using ::atlas::frontend::inertial::InertialState;

struct SessionPoseEntry {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  InertialState x;
  Eigen::Matrix<double, 6, 1> v6 = Eigen::Matrix<double, 6, 1>::Zero();
  double jud = 0.5;
};

struct PreviousSessionSpec {
  std::string name;
  double score_threshold = 0.5;
};

std::vector<PreviousSessionSpec> ParsePreviousSessionList(
    const std::string& text);

bool SaveTrajectoryTum(const std::string& path,
                       const std::vector<InertialState>& poses);
bool LoadTrajectoryTum(const std::string& path,
                       std::vector<InertialState>* poses);

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
bool SaveCloudBin(const std::string& path, const PointObservationList& point_observations);
bool LoadCloudBin(const std::string& path, PointObservationList* point_observations);
bool SaveCloudBin(const std::string& path, const sensor::PointCloudXYZI& cloud);
bool LoadCloudBin(const std::string& path, sensor::PointCloudXYZI* cloud);

}  // namespace serialization
}  // namespace map
}  // namespace atlas

#endif

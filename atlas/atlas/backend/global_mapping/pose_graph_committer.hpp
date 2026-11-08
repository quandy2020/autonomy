/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_BACKEND_GLOBAL_MAPPING_POSE_GRAPH_COMMITTER_HPP_
#define ATLAS_BACKEND_GLOBAL_MAPPING_POSE_GRAPH_COMMITTER_HPP_

#include <vector>

#include "frontend/odometry/scan_pose.hpp"
#include "map/keyframe.hpp"

namespace atlas {
namespace backend {
namespace global_mapping {

/**
 * @brief Validates and atomically applies optimized session pose snapshots.
 * @note The caller must hold the mutex protecting the current session poses.
 */
class PoseGraphCommitter {
 public:
  bool Commit(
      const std::vector<std::vector<frontend::odometry::ScanPosePtr>*>&
          optimized_scans,
      const std::vector<std::vector<map::Keyframe*>*>& optimized_keyframes,
      const std::vector<std::vector<frontend::odometry::ScanPosePtr>*>&
          current_scans,
      const std::vector<std::vector<map::Keyframe*>*>& current_keyframes) const;
};

}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas

#endif

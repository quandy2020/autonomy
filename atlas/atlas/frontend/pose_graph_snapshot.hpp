/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_FRONTEND_POSE_GRAPH_SNAPSHOT_HPP_
#define ATLAS_FRONTEND_POSE_GRAPH_SNAPSHOT_HPP_

#include <cstdint>
#include <memory>
#include <vector>

#include "backend/global_mapping/pgo_edges.hpp"
#include "frontend/inertial/inertial_state.hpp"
#include "frontend/odometry/scan_pose.hpp"
#include "map/keyframe.hpp"

namespace atlas {
namespace frontend {

using ::atlas::frontend::inertial::InertialState;
using ::atlas::frontend::odometry::ScanPosePtr;

/** @brief Owns detached input state for top-down pose graph work.
 */
struct PoseGraphSnapshot {
  std::uint64_t source_pose_version = 0;
  std::uint64_t source_map_state_version = 0;
  int session_id = -1;
  std::vector<std::vector<ScanPosePtr>> scan_poses;
  std::vector<std::vector<std::unique_ptr<map::Keyframe>>> keyframes;
  std::vector<std::vector<ScanPosePtr>*> scan_views;
  std::vector<std::vector<map::Keyframe*>> keyframe_views;
  std::vector<std::vector<map::Keyframe*>*> keyframe_view_pointers;
  backend::global_mapping::PgoEdges loop_edges;
  backend::global_mapping::PgoEdges first_bundle_adjustment_edges;
  backend::global_mapping::PgoEdges second_bundle_adjustment_edges;
  InertialState previous_current_pose;
  bool has_previous_current_pose = false;

  bool empty() const;
  void BuildViews();
};

}  // namespace frontend
}  // namespace atlas

#endif  // ATLAS_FRONTEND_POSE_GRAPH_SNAPSHOT_HPP_

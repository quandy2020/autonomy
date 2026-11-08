/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_BACKEND_GLOBAL_MAPPING_PREVIOUS_SESSION_LOADER_HPP_
#define ATLAS_BACKEND_GLOBAL_MAPPING_PREVIOUS_SESSION_LOADER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "backend/global_mapping/pgo_edges.hpp"
#include "backend/loop_closure/triangle/triangle_loop_detector.hpp"
#include "frontend/odometry/scan_pose.hpp"
#include "map/keyframe.hpp"
#include "map/serialization/voxel_map_serializer.hpp"

namespace atlas {
namespace backend {
namespace global_mapping {

using ::atlas::backend::loop_closure::LoopClosureOptions;
using ::atlas::backend::loop_closure::triangle::TriangleLoopDetector;
using ::atlas::frontend::odometry::ScanPosePtr;
using ::atlas::map::Keyframe;
using ::atlas::map::serialization::SessionPoseEntry;

struct PreviousSessionOptions {
  std::string base_path;
  /** @brief Comma-separated session names and descriptor score thresholds. */
  std::string session_list;
  int window_size = 10;
  int association_size = 10;
  int mapping_stride = 5;
  double voxel_size = 1.0;
  LoopClosureOptions loop_closure;
};

struct PreviousSession {
  std::string name;
  double descriptor_score_threshold = 0.5;
  std::vector<ScanPosePtr> scans;
  std::vector<std::unique_ptr<Keyframe>> keyframes;
  std::unique_ptr<TriangleLoopDetector> loop_detector;
};

/**
 * @brief Load legacy session poses and clouds, then build keyframes and loop
 *        descriptor indexes.
 *
 * Session layout under `base_path/name/`:
 *   - `alidarState.txt` (SaveSessionPoses format)
 *   - `0.bin`, `1.bin`, … body-frame clouds (SaveCloudBin)
 */
class PreviousSessionLoader {
 public:
  /** @brief Load and index history without replacing outputs on failure. */
  bool Load(const PreviousSessionOptions& options,
            std::vector<PreviousSession>* sessions, PgoEdges* edges) const;
};

}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas

#endif  // ATLAS_BACKEND_GLOBAL_MAPPING_PREVIOUS_SESSION_LOADER_HPP_

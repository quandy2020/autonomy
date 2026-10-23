/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_VXSLAM_PREVIOUS_MAP_HPP_
#define ATLAS_VXSLAM_PREVIOUS_MAP_HPP_

#include <memory>
#include <string>
#include <vector>

#include "backend/global_mapping/pgo_edges.hpp"
#include "backend/loop_closure/lidar/triangle_descriptor/triangle_detector.hpp"
#include "frontend/lidar/odometry/scan_pose.hpp"
#include "map/voxel/keyframe.hpp"
#include "map/voxel/serialization/voxel_map_serializer.hpp"

namespace atlas {
namespace vxslam {

struct PreviousMapOptions {
  std::string save_path;
  /** @brief Comma list "name:jud,..." (same as OdometryConfig::previous_map).
   */
  std::string previous_map;
  int win_size = 10;
  int acsize = 10;
  int mgsize = 5;
  double voxel_size = 1.0;
  loop_closure::triangle::TriangleConfig triangle;
};

struct PreviousMapSession {
  std::string name;
  double jud = 0.5;
  std::vector<ScanPosePtr> scan_poses;
  std::vector<Keyframe*> keyframes;
  std::unique_ptr<loop_closure::triangle::TriangleDetector> triangle;
};

/**
 * @brief Load prior sessions: poses + cloud bins + triangle descriptor DB
 *        (Voxel-SLAM previous_map_read, PCL PCD → .bin).
 *
 * Session layout under `save_path/name/`:
 *   - `alidarState.txt` (SaveSessionPoses format)
 *   - `0.bin`, `1.bin`, … body-frame clouds (SaveCloudBin)
 */
bool PreviousMapRead(const PreviousMapOptions& opt,
                     std::vector<PreviousMapSession>* sessions,
                     PgoEdges* edges);

}  // namespace vxslam
}  // namespace atlas

#endif

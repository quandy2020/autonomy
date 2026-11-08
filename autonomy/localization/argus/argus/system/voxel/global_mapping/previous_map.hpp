/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_VXSLAM_PREVIOUS_MAP_HPP_
#define ARGUS_VXSLAM_PREVIOUS_MAP_HPP_

#include <memory>
#include <string>
#include <vector>

#include "argus/matching/loop_closure/triangle_descriptor/triangle_detector.hpp"
#include "argus/system/voxel/global_mapping/pgo_edges.hpp"
#include "argus/system/voxel/odometry/scan_pose.hpp"
#include "argus/system/voxel/serialization/voxel_map_serializer.hpp"
#include "argus/system/voxel/local_mapping/sliding_octree.hpp"

namespace argus {
namespace vxslam {

struct PreviousMapOptions {
    std::string save_path;
    /** @brief Comma list "name:jud,..." (same as OdometryConfig::previous_map). */
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
}  // namespace argus

#endif

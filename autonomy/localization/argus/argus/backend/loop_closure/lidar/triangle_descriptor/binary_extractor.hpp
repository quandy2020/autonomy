/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_MATCHING_BINARY_EXTRACTOR_HPP_
#define ARGUS_MATCHING_BINARY_EXTRACTOR_HPP_

#include <vector>

#include "backend/loop_closure/lidar/triangle_descriptor/triangle_types.hpp"
#include "sensor/point_types.hpp"

namespace argus {
namespace vxslam {
namespace loop_closure {
namespace triangle {

/** @brief Merge coplanar BTC planes (Voxel-SLAM merge_plane /
 * get_project_plane). */
void MergePlanes(const std::vector<TrianglePlane>& origin,
                 const TriangleConfig& cfg, std::vector<TrianglePlane>* merged);

/**
 * @brief Project cloud onto plane slab and extract binary corner descriptors.
 *        Port of STDescManager::extract_binary (no PCL).
 */
void ExtractBinary(const Eigen::Vector3d& project_center,
                   const Eigen::Vector3d& project_normal,
                   const sensor::PointCloudXYZI& cloud,
                   const TriangleConfig& cfg,
                   std::vector<BinaryDescriptor>* binary_list);

/** @brief Non-max suppression on binary corners. */
void NonMaxSuppress(std::vector<BinaryDescriptor>* binary_list,
                    const TriangleConfig& cfg);

/**
 * @brief Top projection planes → binary corners (binary_extractor).
 */
void ExtractCorners(const std::vector<TrianglePlane>& planes,
                    const sensor::PointCloudXYZI& cloud,
                    const TriangleConfig& cfg,
                    std::vector<BinaryDescriptor>* corners);

/** @brief Build STD triangles from binary corners (generate_std). */
void GenerateStdFromBinary(const std::vector<BinaryDescriptor>& corners,
                           int frame_id, const TriangleConfig& cfg,
                           std::vector<TriangleDescriptor>* stds);

}  // namespace triangle
}  // namespace loop_closure
}  // namespace vxslam
}  // namespace argus

#endif

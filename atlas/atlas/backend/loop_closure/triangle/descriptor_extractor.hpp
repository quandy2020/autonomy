/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_BACKEND_LOOP_CLOSURE_TRIANGLE_DESCRIPTOR_EXTRACTOR_HPP_
#define ATLAS_BACKEND_LOOP_CLOSURE_TRIANGLE_DESCRIPTOR_EXTRACTOR_HPP_

#include <vector>

#include "backend/loop_closure/triangle/triangle_descriptor.hpp"
#include "sensor/point_types.hpp"

namespace atlas {
namespace backend {
namespace loop_closure {
namespace triangle {

/** @brief Merge coplanar BTC planes (Voxel-SLAM merge_plane /
 * get_project_plane). */
class DescriptorExtractor {
 public:
  void MergePlanes(const std::vector<TrianglePlane>& origin,
                   const LoopClosureOptions& configuration,
                   std::vector<TrianglePlane>* merged) const;

  /**
   * @brief Project cloud onto plane slab and extract binary corner descriptors.
   *        Port of STDescManager::extract_binary (no PCL).
   */
  void ExtractBinary(const Eigen::Vector3d& project_center,
                     const Eigen::Vector3d& project_normal,
                     const sensor::PointCloudXYZI& cloud,
                     const LoopClosureOptions& configuration,
                     std::vector<BinaryDescriptor>* binary_list) const;

  /** @brief Non-max suppression on binary corners. */
  void NonMaxSuppress(std::vector<BinaryDescriptor>* binary_list,
                      const LoopClosureOptions& configuration) const;

  /**
   * @brief Top projection planes → binary corners.
   */
  void ExtractCorners(const std::vector<TrianglePlane>& planes,
                      const sensor::PointCloudXYZI& cloud,
                      const LoopClosureOptions& configuration,
                      std::vector<BinaryDescriptor>* corners) const;

  /** @brief Build STD triangles from binary corners (generate_std). */
  void GenerateStdFromBinary(
      const std::vector<BinaryDescriptor>& corners, int frame_id,
      const LoopClosureOptions& configuration,
      std::vector<TriangleDescriptor>* descriptors) const;
};

}  // namespace triangle
}  // namespace loop_closure
}  // namespace backend
}  // namespace atlas

#endif  // ATLAS_BACKEND_LOOP_CLOSURE_TRIANGLE_DESCRIPTOR_EXTRACTOR_HPP_

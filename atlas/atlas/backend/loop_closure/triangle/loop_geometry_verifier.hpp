/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_BACKEND_LOOP_CLOSURE_TRIANGLE_LOOP_GEOMETRY_VERIFIER_HPP_
#define ATLAS_BACKEND_LOOP_CLOSURE_TRIANGLE_LOOP_GEOMETRY_VERIFIER_HPP_

#include <Eigen/Core>
#include <unordered_map>
#include <utility>
#include <vector>

#include "backend/loop_closure/triangle/triangle_descriptor.hpp"
#include "sensor/point_types.hpp"

namespace atlas {
namespace backend {
namespace loop_closure {
namespace triangle {

struct TriangleMatchList {
  int match_frame = -1;
  std::pair<int, int> match_id = {-1, -1};
  std::vector<std::pair<TriangleDescriptor, TriangleDescriptor>> match_list;
};

class LoopGeometryVerifier {
 public:
  int64_t HashTriangle(const Eigen::Vector3d& triangle, float resolution) const;

  void SolveTriangle(const TriangleDescriptor& query,
                     const TriangleDescriptor& reference,
                     Eigen::Vector3d* translation,
                     Eigen::Matrix3d* rotation) const;

  double CompareBinaryDescriptors(const BinaryDescriptor& first,
                                  const BinaryDescriptor& second) const;

  /**
   * @brief Vote candidates with 3×3×3 triangle-hash neighbors + binary sim.
   */
  void SelectCandidates(
      const std::vector<TriangleDescriptor>& query,
      const std::unordered_map<int64_t, std::vector<std::pair<int, int>>>&
          hash_table,
      const std::vector<std::vector<TriangleDescriptor>>& descriptors_per_frame,
      const std::vector<int>& frame_ids, int skip_near, int query_frame_id,
      const LoopClosureOptions& configuration,
      std::vector<TriangleMatchList>* candidates) const;

  /**
   * @brief Triangle vote + plane geometric verify (candidate_verify).
   * @return verify_score in [0,1], or -1 on failure.
   */
  double VerifyCandidate(
      const TriangleMatchList& candidate,
      const sensor::PointCloudXYZI& source_plane_cloud,
      const sensor::PointCloudXYZI& target_plane_cloud,
      const LoopClosureOptions& configuration,
      std::pair<Eigen::Vector3d, Eigen::Matrix3d>* relative_pose,
      std::vector<std::pair<TriangleDescriptor, TriangleDescriptor>>*
          successful_matches) const;

  double VerifyPlaneGeometry(
      const sensor::PointCloudXYZI& source,
      const sensor::PointCloudXYZI& target,
      const std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform,
      const LoopClosureOptions& configuration) const;
};

}  // namespace triangle
}  // namespace loop_closure
}  // namespace backend
}  // namespace atlas

#endif  // ATLAS_BACKEND_LOOP_CLOSURE_TRIANGLE_LOOP_GEOMETRY_VERIFIER_HPP_

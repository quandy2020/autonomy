/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_MATCHING_TRIANGLE_VERIFY_HPP_
#define ARGUS_MATCHING_TRIANGLE_VERIFY_HPP_

#include <Eigen/Core>
#include <unordered_map>
#include <utility>
#include <vector>

#include "backend/loop_closure/lidar/triangle_descriptor/triangle_types.hpp"
#include "sensor/point_types.hpp"

namespace argus {
namespace vxslam {
namespace loop_closure {
namespace triangle {

struct TriangleMatchList {
  int match_frame = -1;
  std::pair<int, int> match_id = {-1, -1};
  std::vector<std::pair<TriangleDescriptor, TriangleDescriptor>> match_list;
};

int64_t HashTriangle(const Eigen::Vector3d& tri, float res);

void TriangleSolve(const TriangleDescriptor& query,
                   const TriangleDescriptor& ref, Eigen::Vector3d* t,
                   Eigen::Matrix3d* R);

double BinarySimilarity(const BinaryDescriptor& b1, const BinaryDescriptor& b2);

/**
 * @brief Vote candidates with 3×3×3 triangle-hash neighbors + binary sim.
 */
void CandidateSelector(
    const std::vector<TriangleDescriptor>& query,
    const std::unordered_map<int64_t, std::vector<std::pair<int, int>>>&
        hash_table,
    const std::vector<std::vector<TriangleDescriptor>>& stds_per_frame,
    const std::vector<int>& frame_ids, int skip_near, int query_frame_id,
    const TriangleConfig& cfg, std::vector<TriangleMatchList>* candidates);

/**
 * @brief Triangle vote + plane geometric verify (candidate_verify).
 * @return verify_score in [0,1], or -1 on failure.
 */
double CandidateVerify(
    const TriangleMatchList& candidate, const sensor::PointCloudXYZI& plane_src,
    const sensor::PointCloudXYZI& plane_tar, const TriangleConfig& cfg,
    std::pair<Eigen::Vector3d, Eigen::Matrix3d>* relative,
    std::vector<std::pair<TriangleDescriptor, TriangleDescriptor>>* success);

double PlaneGeometricVerification(
    const sensor::PointCloudXYZI& source, const sensor::PointCloudXYZI& target,
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform,
    const TriangleConfig& cfg);

}  // namespace triangle
}  // namespace loop_closure
}  // namespace vxslam
}  // namespace argus

#endif

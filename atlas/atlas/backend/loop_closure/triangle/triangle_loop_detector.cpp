/*
 * Copyright 2026 The Openbot Authors
 */

#include "backend/loop_closure/triangle/triangle_loop_detector.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "backend/loop_closure/triangle/loop_geometry_verifier.hpp"
#include "map/point_cluster.hpp"
#include "map/voxel_key_hash.hpp"

namespace atlas {
namespace backend {
namespace loop_closure {
namespace triangle {

using ::atlas::map::PointCluster;

TriangleLoopDetector::TriangleLoopDetector(LoopClosureOptions config)
    : config_(std::move(config)) {}

void TriangleLoopDetector::ExtractPlanes(
    const sensor::PointCloudXYZI& cloud,
    std::vector<TrianglePlane>* planes) const {
  if (planes == nullptr) {
    return;
  }
  planes->clear();
  const float vs = config_.voxel_size;
  std::unordered_map<::atlas::map::VoxelKey, PointCluster> voxels;
  for (const auto& p : cloud.points) {
    float loc[3] = {p.x / vs, p.y / vs, p.z / vs};
    for (int j = 0; j < 3; ++j) {
      if (loc[j] < 0) {
        loc[j] -= 1.f;
      }
    }
    const ::atlas::map::VoxelKey key(static_cast<int64_t>(loc[0]),
                                     static_cast<int64_t>(loc[1]),
                                     static_cast<int64_t>(loc[2]));
    voxels[key].push(Eigen::Vector3d(p.x, p.y, p.z));
  }
  int id = 0;
  for (auto& kv : voxels) {
    if (kv.second.N < config_.voxel_init_num) {
      continue;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(kv.second.cov());
    const Eigen::Vector3d ev = saes.eigenvalues();
    if (ev[0] > config_.plane_detection_thre) {
      continue;
    }
    TrianglePlane pl;
    pl.center = kv.second.v / static_cast<double>(kv.second.N);
    pl.normal = saes.eigenvectors().col(0);
    pl.covariance = kv.second.cov();
    pl.min_eigen_value = static_cast<float>(ev[0]);
    pl.radius = static_cast<float>(ev[2]);
    pl.points_size = kv.second.N;
    pl.is_plane = true;
    pl.id = id++;
    pl.d = static_cast<float>(-pl.normal.dot(pl.center));
    planes->push_back(pl);
  }
}

void TriangleLoopDetector::GenerateDescriptors(
    const sensor::PointCloudXYZI& cloud, int frame_id,
    std::vector<TriangleDescriptor>* stds) const {
  GenerateDescriptors(cloud, frame_id, stds, nullptr);
}

void TriangleLoopDetector::GenerateDescriptors(
    const sensor::PointCloudXYZI& cloud, int frame_id,
    std::vector<TriangleDescriptor>* stds,
    sensor::PointCloudXYZI* plane_cloud) const {
  if (stds == nullptr) {
    return;
  }
  stds->clear();
  if (plane_cloud != nullptr) {
    plane_cloud->clear();
  }
  if (cloud.empty()) {
    return;
  }
  std::vector<TrianglePlane> planes;
  ExtractPlanes(cloud, &planes);
  if (plane_cloud != nullptr) {
    for (const TrianglePlane& pl : planes) {
      sensor::PointType pi;
      pi.x = static_cast<float>(pl.center[0]);
      pi.y = static_cast<float>(pl.center[1]);
      pi.z = static_cast<float>(pl.center[2]);
      pi.normal_x = static_cast<float>(pl.normal[0]);
      pi.normal_y = static_cast<float>(pl.normal[1]);
      pi.normal_z = static_cast<float>(pl.normal[2]);
      plane_cloud->push_back(pi);
    }
  }
  std::vector<BinaryDescriptor> corners;
  descriptor_extractor_.ExtractCorners(planes, cloud, config_, &corners);
  if (corners.size() < 3) {
    return;
  }
  descriptor_extractor_.GenerateStdFromBinary(corners, frame_id, config_, stds);
}

void TriangleLoopDetector::IndexFrame(
    int frame_id, const transform::Rigid3d& pose,
    const std::vector<TriangleDescriptor>& stds,
    const sensor::PointCloudXYZI& plane) {
  const int fi = static_cast<int>(frame_ids_.size());
  frame_ids_.push_back(frame_id);
  poses_.push_back(pose);
  descriptors_per_frame_.push_back(stds);
  plane_clouds_.push_back(plane);
  for (int i = 0; i < static_cast<int>(stds.size()); ++i) {
    const int64_t h = geometry_verifier_.HashTriangle(
        stds[static_cast<size_t>(i)].triangle, config_.std_side_resolution);
    hash_table_[h].emplace_back(fi, i);
  }
}

void TriangleLoopDetector::AddDescriptors(
    const std::vector<TriangleDescriptor>& stds) {
  AddDescriptors(stds, sensor::PointCloudXYZI{});
}

void TriangleLoopDetector::AddDescriptors(
    const std::vector<TriangleDescriptor>& stds,
    const sensor::PointCloudXYZI& plane_cloud) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stds.empty()) {
    return;
  }
  IndexFrame(stds.front().frame_number, transform::Rigid3d::Identity(), stds,
             plane_cloud);
}

TriangleLoopDetector::SearchLoopResult TriangleLoopDetector::SearchLoop(
    const std::vector<TriangleDescriptor>& query,
    const sensor::PointCloudXYZI& plane_src, int query_frame_id) {
  SearchLoopResult out;
  if (query.empty()) {
    return out;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<TriangleMatchList> candidates;
  geometry_verifier_.SelectCandidates(
      query, hash_table_, descriptors_per_frame_, frame_ids_,
      config_.skip_near_num, query_frame_id, config_, &candidates);
  double best_score = 0;
  int best_fi = -1;
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> best_tf;
  for (const TriangleMatchList& cand : candidates) {
    const int fi = cand.match_frame;
    if (fi < 0 || fi >= static_cast<int>(plane_clouds_.size())) {
      continue;
    }
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> rel;
    const double score = geometry_verifier_.VerifyCandidate(
        cand, plane_src, plane_clouds_[static_cast<size_t>(fi)], config_, &rel,
        nullptr);
    if (score > best_score) {
      best_score = score;
      best_fi = fi;
      best_tf = rel;
    }
  }
  if (best_score > config_.icp_threshold && best_fi >= 0) {
    out.match_db_index = best_fi;
    out.match_frame_id = frame_ids_[static_cast<size_t>(best_fi)];
    out.score = best_score;
    out.transform = best_tf;
  }
  return out;
}

const sensor::PointCloudXYZI* TriangleLoopDetector::PlaneCloud(
    int db_index) const {
  if (db_index < 0 || db_index >= static_cast<int>(plane_clouds_.size())) {
    return nullptr;
  }
  return &plane_clouds_[static_cast<size_t>(db_index)];
}

std::vector<LoopCandidate> TriangleLoopDetector::Detect(const KeyFrame& kf) {
  std::vector<TriangleDescriptor> q_stds;
  sensor::PointCloudXYZI plane_src;
  GenerateDescriptors(kf.cloud, kf.id, &q_stds, &plane_src);

  const SearchLoopResult result = SearchLoop(q_stds, plane_src, kf.id);

  std::vector<LoopCandidate> out;
  if (result.ok()) {
    LoopCandidate c;
    c.query_id = kf.id;
    c.match_id = result.match_frame_id;
    c.score = result.score;
    c.T_query_match = transform::Rigid3d(
        result.transform.first, Eigen::Quaterniond(result.transform.second));
    out.push_back(c);
  }
  AddDescriptors(q_stds, plane_src);
  // Keep pose from keyframe (AddDescriptors uses Identity by default).
  if (!q_stds.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!poses_.empty()) {
      poses_.back() = kf.T_w_b;
    }
  }
  return out;
}

}  // namespace triangle
}  // namespace loop_closure
}  // namespace backend
}  // namespace atlas

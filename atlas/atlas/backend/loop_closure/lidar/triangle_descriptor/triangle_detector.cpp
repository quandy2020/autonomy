/*
 * Copyright 2026 The Openbot Authors
 */

#include "backend/loop_closure/lidar/triangle_descriptor/triangle_detector.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "backend/loop_closure/lidar/triangle_descriptor/binary_extractor.hpp"
#include "backend/loop_closure/lidar/triangle_descriptor/triangle_verify.hpp"
#include "map/voxel/point_cluster.hpp"
#include "map/voxel/voxel_key_hash.hpp"

namespace atlas {
namespace vxslam {
namespace loop_closure {
namespace triangle {

TriangleDetector::TriangleDetector(TriangleConfig config)
    : config_(std::move(config)) {}

void TriangleDetector::ExtractPlanes(const sensor::PointCloudXYZI& cloud,
                                     std::vector<TrianglePlane>* planes) const {
  if (planes == nullptr) {
    return;
  }
  planes->clear();
  const float vs = config_.voxel_size;
  std::unordered_map<map::voxel::VoxelKey, PointCluster> voxels;
  for (const auto& p : cloud.points) {
    float loc[3] = {p.x / vs, p.y / vs, p.z / vs};
    for (int j = 0; j < 3; ++j) {
      if (loc[j] < 0) {
        loc[j] -= 1.f;
      }
    }
    const map::voxel::VoxelKey key(static_cast<int64_t>(loc[0]),
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

void TriangleDetector::GenerateDescriptors(
    const sensor::PointCloudXYZI& cloud, int frame_id,
    std::vector<TriangleDescriptor>* stds) const {
  GenerateDescriptors(cloud, frame_id, stds, nullptr);
}

void TriangleDetector::GenerateDescriptors(
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
  ExtractCorners(planes, cloud, config_, &corners);
  if (corners.size() < 3) {
    if (planes.size() < 3) {
      return;
    }
    const int n = static_cast<int>(planes.size());
    const int near =
        std::min(n - 1, static_cast<int>(config_.descriptor_near_num));
    for (int i = 0; i < n; ++i) {
      std::vector<std::pair<double, int>> dist;
      for (int j = 0; j < n; ++j) {
        if (i == j) {
          continue;
        }
        dist.emplace_back((planes[static_cast<size_t>(i)].center -
                           planes[static_cast<size_t>(j)].center)
                              .norm(),
                          j);
      }
      std::partial_sort(
          dist.begin(),
          dist.begin() + std::min(near, static_cast<int>(dist.size())),
          dist.end(),
          [](const auto& a, const auto& b) { return a.first < b.first; });
      const int use = std::min(near, static_cast<int>(dist.size()));
      for (int a = 0; a < use; ++a) {
        for (int b = a + 1; b < use; ++b) {
          const Eigen::Vector3d& A = planes[static_cast<size_t>(i)].center;
          const Eigen::Vector3d& B =
              planes[static_cast<size_t>(dist[static_cast<size_t>(a)].second)]
                  .center;
          const Eigen::Vector3d& C =
              planes[static_cast<size_t>(dist[static_cast<size_t>(b)].second)]
                  .center;
          Eigen::Vector3d sides((A - B).norm(), (B - C).norm(), (C - A).norm());
          if (sides.minCoeff() < config_.descriptor_min_len ||
              sides.maxCoeff() > config_.descriptor_max_len) {
            continue;
          }
          std::sort(sides.data(), sides.data() + 3);
          TriangleDescriptor s;
          s.triangle = sides;
          s.center = (A + B + C) / 3.0;
          s.frame_number = frame_id;
          s.binary_a.location = A;
          s.binary_b.location = B;
          s.binary_c.location = C;
          stds->push_back(s);
        }
      }
    }
    return;
  }
  GenerateStdFromBinary(corners, frame_id, config_, stds);
}

void TriangleDetector::IndexFrame(int frame_id, const transform::Rigid3d& pose,
                                  const std::vector<TriangleDescriptor>& stds,
                                  const sensor::PointCloudXYZI& plane) {
  const int fi = static_cast<int>(frame_ids_.size());
  frame_ids_.push_back(frame_id);
  poses_.push_back(pose);
  descriptors_per_frame_.push_back(stds);
  plane_clouds_.push_back(plane);
  for (int i = 0; i < static_cast<int>(stds.size()); ++i) {
    const int64_t h = HashTriangle(stds[static_cast<size_t>(i)].triangle,
                                   config_.std_side_resolution);
    hash_table_[h].emplace_back(fi, i);
  }
}

void TriangleDetector::AddDescriptors(
    const std::vector<TriangleDescriptor>& stds) {
  AddDescriptors(stds, sensor::PointCloudXYZI{});
}

void TriangleDetector::AddDescriptors(
    const std::vector<TriangleDescriptor>& stds,
    const sensor::PointCloudXYZI& plane_cloud) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (stds.empty()) {
    return;
  }
  IndexFrame(stds.front().frame_number, transform::Rigid3d::Identity(), stds,
             plane_cloud);
}

TriangleDetector::SearchLoopResult TriangleDetector::SearchLoop(
    const std::vector<TriangleDescriptor>& query,
    const sensor::PointCloudXYZI& plane_src, int query_frame_id) {
  SearchLoopResult out;
  if (query.empty()) {
    return out;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<TriangleMatchList> candidates;
  CandidateSelector(query, hash_table_, descriptors_per_frame_, frame_ids_,
                    config_.skip_near_num, query_frame_id, config_,
                    &candidates);
  double best_score = 0;
  int best_fi = -1;
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> best_tf;
  for (const TriangleMatchList& cand : candidates) {
    const int fi = cand.match_frame;
    if (fi < 0 || fi >= static_cast<int>(plane_clouds_.size())) {
      continue;
    }
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> rel;
    const double score =
        CandidateVerify(cand, plane_src, plane_clouds_[static_cast<size_t>(fi)],
                        config_, &rel, nullptr);
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

const sensor::PointCloudXYZI* TriangleDetector::PlaneCloud(int db_index) const {
  if (db_index < 0 || db_index >= static_cast<int>(plane_clouds_.size())) {
    return nullptr;
  }
  return &plane_clouds_[static_cast<size_t>(db_index)];
}

std::vector<LoopCandidate> TriangleDetector::Detect(const KeyFrame& kf) {
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
}  // namespace vxslam
}  // namespace atlas

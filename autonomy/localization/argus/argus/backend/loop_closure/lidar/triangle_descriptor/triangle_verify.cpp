/*
 * Copyright 2026 The Openbot Authors
 */
#include "backend/loop_closure/lidar/triangle_descriptor/triangle_verify.hpp"

#include <Eigen/LU>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>

#include "common/math/kdtree.hpp"

namespace argus {
namespace vxslam {
namespace loop_closure {
namespace triangle {

int64_t HashTriangle(const Eigen::Vector3d& tri, float res) {
  const int64_t a = static_cast<int64_t>(std::round(tri[0] / res));
  const int64_t b = static_cast<int64_t>(std::round(tri[1] / res));
  const int64_t c = static_cast<int64_t>(std::round(tri[2] / res));
  constexpr int64_t kP = 116101;
  return ((a * kP + b) * kP + c);
}

void TriangleSolve(const TriangleDescriptor& query,
                   const TriangleDescriptor& ref, Eigen::Vector3d* t,
                   Eigen::Matrix3d* R) {
  if (t == nullptr || R == nullptr) {
    return;
  }
  Eigen::Matrix3d src = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dst = Eigen::Matrix3d::Zero();
  src.col(0) = query.binary_a.location - query.center;
  src.col(1) = query.binary_b.location - query.center;
  src.col(2) = query.binary_c.location - query.center;
  dst.col(0) = ref.binary_a.location - ref.center;
  dst.col(1) = ref.binary_b.location - ref.center;
  dst.col(2) = ref.binary_c.location - ref.center;
  const Eigen::Matrix3d covariance = src * dst.transpose();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      covariance, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d U = svd.matrixU();
  *R = V * U.transpose();
  if (R->determinant() < 0) {
    Eigen::Matrix3d K;
    K << 1, 0, 0, 0, 1, 0, 0, 0, -1;
    *R = V * K * U.transpose();
  }
  *t = -(*R) * query.center + ref.center;
}

double BinarySimilarity(const BinaryDescriptor& b1,
                        const BinaryDescriptor& b2) {
  double dis = 0;
  const size_t n = std::min(b1.occupy_array.size(), b2.occupy_array.size());
  for (size_t i = 0; i < n; ++i) {
    if (b1.occupy_array[i] && b2.occupy_array[i]) {
      dis += 1;
    }
  }
  const double s =
      static_cast<double>(b1.summary) + static_cast<double>(b2.summary);
  if (s < 1e-9) {
    return 0;
  }
  return 2.0 * dis / s;
}

void CandidateSelector(
    const std::vector<TriangleDescriptor>& query,
    const std::unordered_map<int64_t, std::vector<std::pair<int, int>>>&
        hash_table,
    const std::vector<std::vector<TriangleDescriptor>>& stds_per_frame,
    const std::vector<int>& frame_ids, int skip_near, int query_frame_id,
    const TriangleConfig& cfg, std::vector<TriangleMatchList>* candidates) {
  if (candidates == nullptr) {
    return;
  }
  candidates->clear();
  if (stds_per_frame.empty()) {
    return;
  }
  std::vector<double> match_array(stds_per_frame.size(), 0.0);
  std::vector<bool> useful_match(query.size(), false);
  std::vector<std::vector<int64_t>> useful_pos(query.size());
  std::vector<std::vector<size_t>> useful_idx(query.size());

  std::vector<Eigen::Vector3i> voxel_round;
  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      for (int z = -1; z <= 1; ++z) {
        voxel_round.emplace_back(x, y, z);
      }
    }
  }

  for (size_t i = 0; i < query.size(); ++i) {
    const TriangleDescriptor& descriptor = query[i];
    const double dis_threshold =
        descriptor.triangle.norm() * cfg.rough_dis_threshold;
    for (const auto& inc : voxel_round) {
      const Eigen::Vector3d tri =
          descriptor.triangle + Eigen::Vector3d(inc[0], inc[1], inc[2]);
      const Eigen::Vector3d voxel_center(std::floor(tri[0]) + 0.5,
                                         std::floor(tri[1]) + 0.5,
                                         std::floor(tri[2]) + 0.5);
      if ((descriptor.triangle - voxel_center).norm() >= 1.5) {
        continue;
      }
      const int64_t h = HashTriangle(tri, cfg.std_side_resolution);
      auto iter = hash_table.find(h);
      if (iter == hash_table.end()) {
        continue;
      }
      for (size_t j = 0; j < iter->second.size(); ++j) {
        const int fi = iter->second[j].first;
        const int si = iter->second[j].second;
        if (fi < 0 || fi >= static_cast<int>(stds_per_frame.size())) {
          continue;
        }
        const TriangleDescriptor& ref =
            stds_per_frame[static_cast<size_t>(fi)][static_cast<size_t>(si)];
        if (std::abs(query_frame_id - ref.frame_number) <= skip_near) {
          continue;
        }
        if ((descriptor.triangle - ref.triangle).norm() >= dis_threshold) {
          continue;
        }
        const double similarity =
            (BinarySimilarity(descriptor.binary_a, ref.binary_a) +
             BinarySimilarity(descriptor.binary_b, ref.binary_b) +
             BinarySimilarity(descriptor.binary_c, ref.binary_c)) /
            3.0;
        if (similarity > cfg.similarity_threshold) {
          useful_match[i] = true;
          useful_pos[i].push_back(h);
          useful_idx[i].push_back(j);
        }
      }
    }
  }

  struct Rec {
    size_t qi;
    int64_t h;
    size_t hj;
    int fi;
  };
  std::vector<Rec> recs;
  for (size_t i = 0; i < useful_match.size(); ++i) {
    if (!useful_match[i]) {
      continue;
    }
    for (size_t j = 0; j < useful_idx[i].size(); ++j) {
      const int64_t h = useful_pos[i][j];
      auto it = hash_table.find(h);
      if (it == hash_table.end()) {
        continue;
      }
      const auto& pr = it->second[useful_idx[i][j]];
      match_array[static_cast<size_t>(pr.first)] += 1;
      recs.push_back({i, h, useful_idx[i][j], pr.first});
    }
  }
  (void)frame_ids;

  for (int cnt = 0; cnt < cfg.candidate_num; ++cnt) {
    auto max_it = std::max_element(match_array.begin(), match_array.end());
    const int max_vote_index = static_cast<int>(max_it - match_array.begin());
    const int max_vote = static_cast<int>(*max_it);
    if (max_vote_index < 0 || max_vote < 5) {
      break;
    }
    match_array[static_cast<size_t>(max_vote_index)] = 0;
    TriangleMatchList match_triangle_list;
    // match_id.second = DB frame index (plane_cloud_vec_ / stds index).
    match_triangle_list.match_frame = max_vote_index;
    match_triangle_list.match_id = {query_frame_id, max_vote_index};
    for (const Rec& r : recs) {
      if (r.fi != max_vote_index) {
        continue;
      }
      auto it = hash_table.find(r.h);
      if (it == hash_table.end()) {
        continue;
      }
      const auto& pr = it->second[r.hj];
      const TriangleDescriptor& ref =
          stds_per_frame[static_cast<size_t>(pr.first)]
                        [static_cast<size_t>(pr.second)];
      match_triangle_list.match_list.emplace_back(query[r.qi], ref);
    }
    candidates->push_back(std::move(match_triangle_list));
  }
}

double PlaneGeometricVerification(
    const sensor::PointCloudXYZI& source, const sensor::PointCloudXYZI& target,
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform,
    const TriangleConfig& cfg) {
  if (source.empty() || target.empty()) {
    return -1.0;
  }
  std::vector<Eigen::Vector3d> tar_pts;
  tar_pts.reserve(target.size());
  for (const auto& p : target.points) {
    tar_pts.emplace_back(p.x, p.y, p.z);
  }
  math::KdTree kd;
  kd.SetInput(tar_pts);
  double useful_match = 0;
  const double normal_threshold = cfg.normal_threshold;
  const double dis_threshold = cfg.dis_threshold;
  const Eigen::Vector3d& t = transform.first;
  const Eigen::Matrix3d& rot = transform.second;
  for (const auto& sp : source.points) {
    Eigen::Vector3d pi(sp.x, sp.y, sp.z);
    pi = rot * pi + t;
    Eigen::Vector3d ni(sp.normal_x, sp.normal_y, sp.normal_z);
    ni = rot * ni;
    std::vector<int> idx;
    std::vector<float> dist2;
    if (kd.NearestKSearch(pi, 1, &idx, &dist2) <= 0) {
      continue;
    }
    const auto& np = target.points[static_cast<size_t>(idx[0])];
    const Eigen::Vector3d tpi(np.x, np.y, np.z);
    const Eigen::Vector3d tni(np.normal_x, np.normal_y, np.normal_z);
    const Eigen::Vector3d normal_inc = ni - tni;
    const Eigen::Vector3d normal_add = ni + tni;
    const double point_to_plane = std::fabs(tni.dot(pi - tpi));
    if ((normal_inc.norm() < normal_threshold ||
         normal_add.norm() < normal_threshold) &&
        point_to_plane < dis_threshold) {
      useful_match++;
    }
  }
  return useful_match / static_cast<double>(source.size());
}

double CandidateVerify(
    const TriangleMatchList& candidate, const sensor::PointCloudXYZI& plane_src,
    const sensor::PointCloudXYZI& plane_tar, const TriangleConfig& cfg,
    std::pair<Eigen::Vector3d, Eigen::Matrix3d>* relative,
    std::vector<std::pair<TriangleDescriptor, TriangleDescriptor>>* success) {
  if (relative == nullptr) {
    return -1;
  }
  if (success) {
    success->clear();
  }
  if (candidate.match_list.empty()) {
    return -1;
  }
  constexpr double kDisTh = 3.0;
  const int skip_len = static_cast<int>(candidate.match_list.size() / 50) + 1;
  const int use_size = static_cast<int>(candidate.match_list.size()) / skip_len;
  int max_vote = 0;
  Eigen::Matrix3d best_rot = Eigen::Matrix3d::Identity();
  Eigen::Vector3d best_t = Eigen::Vector3d::Zero();

  for (int i = 0; i < use_size; ++i) {
    const auto& single_pair =
        candidate.match_list[static_cast<size_t>(i * skip_len)];
    Eigen::Vector3d test_t;
    Eigen::Matrix3d test_rot;
    TriangleSolve(single_pair.first, single_pair.second, &test_t, &test_rot);
    int vote = 0;
    for (const auto& verify_pair : candidate.match_list) {
      const Eigen::Vector3d A_t =
          test_rot * verify_pair.first.binary_a.location + test_t;
      if ((A_t - verify_pair.second.binary_a.location).norm() >= kDisTh) {
        continue;
      }
      const Eigen::Vector3d B_t =
          test_rot * verify_pair.first.binary_b.location + test_t;
      if ((B_t - verify_pair.second.binary_b.location).norm() >= kDisTh) {
        continue;
      }
      const Eigen::Vector3d C_t =
          test_rot * verify_pair.first.binary_c.location + test_t;
      if ((C_t - verify_pair.second.binary_c.location).norm() < kDisTh) {
        ++vote;
      }
    }
    if (vote > max_vote) {
      max_vote = vote;
      best_rot = test_rot;
      best_t = test_t;
    }
  }
  if (max_vote < 4) {
    return -1;
  }
  relative->first = best_t;
  relative->second = best_rot;
  return PlaneGeometricVerification(plane_src, plane_tar, *relative, cfg);
}

}  // namespace triangle
}  // namespace loop_closure
}  // namespace vxslam
}  // namespace argus

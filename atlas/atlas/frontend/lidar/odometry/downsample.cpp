/*
 * Copyright 2026 The Openbot Authors
 */
#include "frontend/lidar/odometry/downsample.hpp"

#include <Eigen/Core>
#include <unordered_map>
#include <utility>
#include <vector>

#include "map/voxel/voxel_key_hash.hpp"

namespace atlas {
namespace vxslam {

using map::voxel::VoxelKey;

double imu_coef = 1e-4;

void DownsampleVoxel(sensor::PointCloudXYZI* cloud, double voxel_size) {
  if (cloud == nullptr || voxel_size < 0.001) {
    return;
  }
  std::unordered_map<VoxelKey, sensor::PointType> feat_map;
  for (const sensor::PointType& p_c : cloud->points) {
    float loc[3] = {p_c.x / static_cast<float>(voxel_size),
                    p_c.y / static_cast<float>(voxel_size),
                    p_c.z / static_cast<float>(voxel_size)};
    for (int j = 0; j < 3; ++j) {
      if (loc[j] < 0) {
        loc[j] -= 1.0f;
      }
    }
    const VoxelKey key(static_cast<int64_t>(loc[0]),
                       static_cast<int64_t>(loc[1]),
                       static_cast<int64_t>(loc[2]));
    auto it = feat_map.find(key);
    if (it == feat_map.end()) {
      sensor::PointType pp = p_c;
      pp.curvature = 1.f;
      feat_map[key] = pp;
    } else {
      sensor::PointType& pp = it->second;
      pp.x = (pp.x * pp.curvature + p_c.x) / (pp.curvature + 1.f);
      pp.y = (pp.y * pp.curvature + p_c.y) / (pp.curvature + 1.f);
      pp.z = (pp.z * pp.curvature + p_c.z) / (pp.curvature + 1.f);
      pp.curvature += 1.f;
    }
  }
  cloud->clear();
  cloud->reserve(feat_map.size());
  for (auto& kv : feat_map) {
    cloud->push_back(kv.second);
  }
}

void DownsampleClose(sensor::PointCloudXYZI* cloud, double voxel_size) {
  if (cloud == nullptr || voxel_size < 0.001) {
    return;
  }
  std::unordered_map<VoxelKey, std::vector<sensor::PointType>> feat_map;
  for (const sensor::PointType& p_c : cloud->points) {
    float loc[3] = {p_c.x / static_cast<float>(voxel_size),
                    p_c.y / static_cast<float>(voxel_size),
                    p_c.z / static_cast<float>(voxel_size)};
    for (int j = 0; j < 3; ++j) {
      if (loc[j] < 0) {
        loc[j] -= 1.0f;
      }
    }
    const VoxelKey key(static_cast<int64_t>(loc[0]),
                       static_cast<int64_t>(loc[1]),
                       static_cast<int64_t>(loc[2]));
    feat_map[key].push_back(p_c);
  }
  cloud->clear();
  cloud->reserve(feat_map.size());
  for (auto& kv : feat_map) {
    auto& pts = kv.second;
    if (pts.empty()) {
      continue;
    }
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    for (const auto& p : pts) {
      c += Eigen::Vector3d(p.x, p.y, p.z);
    }
    c /= static_cast<double>(pts.size());
    double best = 1e100;
    size_t best_i = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
      const double d =
          (Eigen::Vector3d(pts[i].x, pts[i].y, pts[i].z) - c).squaredNorm();
      if (d < best) {
        best = d;
        best_i = i;
      }
    }
    cloud->push_back(pts[best_i]);
  }
}

void DownsamplePointVars(const PointVarList& points, double voxel_size,
                         sensor::PointCloudXYZI* out) {
  if (out == nullptr || voxel_size < 0.001) {
    return;
  }
  std::unordered_map<VoxelKey, std::pair<PointVar, int>> feat_map;
  for (const PointVar& pv : points) {
    float loc[3] = {static_cast<float>(pv.pnt[0] / voxel_size),
                    static_cast<float>(pv.pnt[1] / voxel_size),
                    static_cast<float>(pv.pnt[2] / voxel_size)};
    for (int j = 0; j < 3; ++j) {
      if (loc[j] < 0) {
        loc[j] -= 1.0f;
      }
    }
    const VoxelKey key(static_cast<int64_t>(loc[0]),
                       static_cast<int64_t>(loc[1]),
                       static_cast<int64_t>(loc[2]));
    auto it = feat_map.find(key);
    if (it == feat_map.end()) {
      feat_map[key] = std::make_pair(pv, 1);
    } else {
      auto& pp = it->second;
      pp.first.pnt = (pp.first.pnt * pp.second + pv.pnt) / (pp.second + 1);
      pp.first.var = (pp.first.var * pp.second + pv.var) / (pp.second + 1);
      pp.second += 1;
    }
  }
  out->clear();
  out->reserve(feat_map.size());
  for (auto& kv : feat_map) {
    const PointVar& pv = kv.second.first;
    sensor::PointType ap;
    ap.x = static_cast<float>(pv.pnt[0]);
    ap.y = static_cast<float>(pv.pnt[1]);
    ap.z = static_cast<float>(pv.pnt[2]);
    ap.normal_x = static_cast<float>(pv.var(0, 0));
    ap.normal_y = static_cast<float>(pv.var(1, 1));
    ap.normal_z = static_cast<float>(pv.var(2, 2));
    out->push_back(ap);
  }
}

}  // namespace vxslam
}  // namespace atlas

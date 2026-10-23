/*
 * Copyright 2026 The Openbot Authors
 */
#include "map/voxel/keyframe_load.hpp"

#include <algorithm>

#include "common/math/kdtree.hpp"
#include "frontend/lidar/odometry/point_var.hpp"

namespace atlas {
namespace vxslam {

bool KeyframeLoading(std::unordered_map<VoxelKey, SlidingOctree*>* feat_map,
                     const std::vector<Keyframe*>& keyframes,
                     const Eigen::Vector3d& curr_p, double radius_m,
                     int win_size, int* history_kfsize, double jour) {
  if (feat_map == nullptr || history_kfsize == nullptr ||
      *history_kfsize <= 0 || keyframes.empty() || win_size <= 0) {
    return false;
  }
  std::vector<Eigen::Vector3d> centers;
  centers.reserve(keyframes.size());
  for (const Keyframe* kf : keyframes) {
    if (kf == nullptr) {
      centers.emplace_back(1e9, 1e9, 1e9);
    } else {
      centers.push_back(kf->x0.p);
    }
  }
  math::KdTree kd;
  kd.SetInput(centers);
  std::vector<int> idx;
  std::vector<float> dist2;
  const int k = std::min(16, static_cast<int>(centers.size()));
  if (kd.NearestKSearch(curr_p, k, &idx, &dist2) <= 0) {
    return false;
  }
  const double r2 = radius_m * radius_m;
  for (size_t n = 0; n < idx.size(); ++n) {
    if (static_cast<double>(dist2[n]) > r2) {
      continue;
    }
    const int id = idx[n];
    if (id < 0 || id >= static_cast<int>(keyframes.size())) {
      continue;
    }
    Keyframe* kf = keyframes[static_cast<size_t>(id)];
    if (kf == nullptr || !kf->exist || kf->plptr == nullptr) {
      continue;
    }
    PointVarList pvec;
    pvec.reserve(kf->plptr->size());
    PointVar pv;
    pv.var.setZero();
    for (const PointType& ap : kf->plptr->points) {
      pv.pnt << ap.x, ap.y, ap.z;
      pv.pnt = kf->x0.R * pv.pnt + kf->x0.p;
      pvec.push_back(pv);
    }
    CutVoxel(*feat_map, pvec, win_size, jour);
    kf->exist = 0;
    --(*history_kfsize);
    return true;
  }
  return false;
}

}  // namespace vxslam
}  // namespace atlas

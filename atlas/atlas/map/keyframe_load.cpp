/*
 * Copyright 2026 The Openbot Authors
 */
#include "map/keyframe_load.hpp"

#include <algorithm>

#include "common/math/kdtree.hpp"
#include "map/point_observation.hpp"

namespace atlas {
namespace map {

using ::atlas::map::PointObservation;
using ::atlas::map::PointObservationList;

bool KeyframeLoading(
    SlidingVoxelMap* feat_map, const std::vector<Keyframe*>& keyframes,
    const Eigen::Vector3d& curr_p, double radius_m, int win_size,
    int* history_kfsize, double jour,
    std::shared_ptr<const SlidingVoxelMapOptions> map_options) {
  if (feat_map == nullptr || history_kfsize == nullptr ||
      *history_kfsize <= 0 || keyframes.empty() || win_size <= 0 ||
      map_options == nullptr) {
    return false;
  }
  std::vector<Eigen::Vector3d> centers;
  centers.reserve(keyframes.size());
  for (const Keyframe* kf : keyframes) {
    if (kf == nullptr) {
      centers.emplace_back(1e9, 1e9, 1e9);
    } else {
      centers.push_back(kf->inertial_state.p);
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
    if (kf == nullptr || !kf->map_insertion_pending ||
        kf->point_cloud == nullptr) {
      continue;
    }
    PointObservationList point_observations;
    point_observations.reserve(kf->point_cloud->size());
    PointObservation point_observation;
    point_observation.covariance.setZero();
    for (const PointType& ap : kf->point_cloud->points) {
      point_observation.position << ap.x, ap.y, ap.z;
      point_observation.position =
          kf->inertial_state.R * point_observation.position +
          kf->inertial_state.p;
      point_observations.push_back(point_observation);
    }
    CutVoxel(*feat_map, point_observations, win_size, jour, map_options);
    kf->map_insertion_pending = false;
    --(*history_kfsize);
    return true;
  }
  return false;
}

}  // namespace map
}  // namespace atlas

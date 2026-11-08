/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_MAP_KEYFRAME_HPP_
#define ATLAS_MAP_KEYFRAME_HPP_

#include <memory>

#include "frontend/inertial/inertial_state.hpp"
#include "sensor/point_types.hpp"

namespace atlas {
namespace map {

using ::atlas::frontend::inertial::InertialState;

// A keyframe stores its inertial state and immutable body-frame point cloud.
// Shared ownership lets background optimization retain a consistent snapshot.
struct Keyframe {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  InertialState inertial_state;
  std::shared_ptr<const sensor::PointCloudXYZI> point_cloud;
  bool map_insertion_pending = false;
  int keyframe_id = -1;
  int session_id = -1;
  double trajectory_distance_meters = 0.0;

  explicit Keyframe(const InertialState& initial_state)
      : inertial_state(initial_state),
        point_cloud(std::make_shared<const sensor::PointCloudXYZI>()) {}
};

}  // namespace map
}  // namespace atlas

#endif  // ATLAS_MAP_KEYFRAME_HPP_

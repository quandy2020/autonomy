/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_MAP_VOXEL_KEYFRAME_HPP_
#define ATLAS_MAP_VOXEL_KEYFRAME_HPP_

#include <memory>

#include "frontend/lidar/imu/imu_state.hpp"
#include "sensor/point_types.hpp"

namespace atlas {
namespace vxslam {

// A keyframe stores its state and body-frame point cloud. Shared cloud
// ownership supports history maps; callers synchronize mutable access.
struct Keyframe {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuState x0;
  std::shared_ptr<sensor::PointCloudXYZI> plptr;
  int exist = 0;
  int id = -1;
  int mp = -1;
  float jour = 0.0f;

  explicit Keyframe(const ImuState& initial_state) : x0(initial_state) {
    plptr = std::make_shared<sensor::PointCloudXYZI>();
  }

  void generate(sensor::PointCloudXYZI& pl_send,
                Eigen::Matrix3d rot = Eigen::Matrix3d::Identity(),
                Eigen::Vector3d tra = Eigen::Vector3d(0, 0, 0)) {
    Eigen::Vector3d v3;
    for (sensor::PointType ap : plptr->points) {
      v3 << ap.x, ap.y, ap.z;
      v3 = rot * v3 + tra;
      ap.x = v3[0];
      ap.y = v3[1];
      ap.z = v3[2];
      pl_send.push_back(ap);
    }
  }
};

}  // namespace vxslam
}  // namespace atlas

#endif  // ATLAS_MAP_VOXEL_KEYFRAME_HPP_

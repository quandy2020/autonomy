/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_MAP_POINT_OBSERVATION_HPP_
#define ATLAS_MAP_POINT_OBSERVATION_HPP_

#include <Eigen/Core>
#include <memory>
#include <vector>

namespace atlas {
namespace map {

struct PointObservation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
};

using PointObservationList =
    std::vector<PointObservation, Eigen::aligned_allocator<PointObservation>>;
using SharedPointObservationList = std::shared_ptr<PointObservationList>;

}  // namespace map
}  // namespace atlas

#endif

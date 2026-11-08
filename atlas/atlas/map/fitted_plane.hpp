/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_MAP_FITTED_PLANE_HPP_
#define ATLAS_MAP_FITTED_PLANE_HPP_

#include <Eigen/Core>

namespace atlas {
namespace map {

struct FittedPlane {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
  float radius = 0.0F;
  bool is_plane = false;
};

}  // namespace map
}  // namespace atlas

#endif

/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_MAP_POINT_COVARIANCE_HPP_
#define ATLAS_MAP_POINT_COVARIANCE_HPP_

#include <Eigen/Core>

namespace atlas {
namespace map {

/** @brief Propagates point and pose covariance into a map point covariance. */
inline void ComputePointTransformCovariance(
    const Eigen::Matrix3d& point_covariance,
    const Eigen::Vector3d& transformed_point,
    Eigen::Matrix<double, 9, 9>* transformed_covariance) {
  Eigen::Matrix<double, 6, 3> point_jacobian;
  point_jacobian << 2 * transformed_point(0), 0, 0, transformed_point(1),
      transformed_point(0), 0, transformed_point(2), 0, transformed_point(0), 0,
      2 * transformed_point(1), 0, 0, transformed_point(2),
      transformed_point(1), 0, 0, 2 * transformed_point(2);

  const Eigen::Matrix<double, 6, 3> weighted_jacobian =
      point_jacobian * point_covariance;
  transformed_covariance->block<6, 6>(0, 0) =
      weighted_jacobian * point_jacobian.transpose();
  transformed_covariance->block<6, 3>(0, 6) = weighted_jacobian;
  transformed_covariance->block<3, 6>(6, 0) = weighted_jacobian.transpose();
  transformed_covariance->block<3, 3>(6, 6) = point_covariance;
}

}  // namespace map
}  // namespace atlas

#endif  // ATLAS_MAP_POINT_COVARIANCE_HPP_

/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_VXSLAM_POINT_VAR_HPP_
#define ARGUS_VXSLAM_POINT_VAR_HPP_

#include <Eigen/Core>
#include <memory>
#include <vector>

namespace argus {
namespace vxslam {

/** @brief 3D point with 3×3 covariance. */
struct PointVar {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d pnt = Eigen::Vector3d::Zero();
  Eigen::Matrix3d var = Eigen::Matrix3d::Identity();
};

using PointVarList = std::vector<PointVar, Eigen::aligned_allocator<PointVar>>;
using PointVarListPtr = std::shared_ptr<PointVarList>;

/** @brief Fitted plane in a voxel. */
struct Plane {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 6, 6> plane_var = Eigen::Matrix<double, 6, 6>::Zero();
  float radius = 0.f;
  bool is_plane = false;
};

}  // namespace vxslam
}  // namespace argus

#endif

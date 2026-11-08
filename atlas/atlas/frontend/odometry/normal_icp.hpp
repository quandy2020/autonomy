/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file normal_icp.hpp
 * @brief Normal-consistent point-to-plane ICP (Voxel-SLAM loop_refine).
 */

#ifndef ATLAS_FRONTEND_ODOMETRY_NORMAL_ICP_HPP_
#define ATLAS_FRONTEND_ODOMETRY_NORMAL_ICP_HPP_

#include <Eigen/Core>
#include <utility>

#include "sensor/point_types.hpp"

namespace atlas {
namespace frontend {
namespace odometry {

/**
 * @brief Refine (t, R) aligning src → tar with normal gating.
 * @param pl_src Source cloud (normals in normal_*).
 * @param pl_tar Target cloud.
 * @param pose Pair (translation, rotation); updated in place.
 * @param icp_eigval Min eigenvalue of normal Gram matrix for acceptance.
 * @return True when converged and eig_min > icp_eigval.
 */
bool NormalIcp(const sensor::PointCloudXYZI& pl_src,
               const sensor::PointCloudXYZI& pl_tar,
               std::pair<Eigen::Vector3d, Eigen::Matrix3d>* pose,
               double icp_eigval);

}  // namespace odometry
}  // namespace frontend
}  // namespace atlas

#endif  // ATLAS_FRONTEND_ODOMETRY_NORMAL_ICP_HPP_

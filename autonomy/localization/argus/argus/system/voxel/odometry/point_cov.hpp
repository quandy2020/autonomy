/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_VXSLAM_POINT_COV_HPP_
#define ARGUS_VXSLAM_POINT_COV_HPP_

#include <cmath>

#include <Eigen/Core>

#include "argus/sensor/point_types.hpp"
#include "argus/system/voxel/odometry/imu_state.hpp"
#include "argus/system/voxel/odometry/point_var.hpp"

namespace argus {
namespace vxslam {

/**
 * @brief Body-frame range/beam covariance (Voxel-SLAM calcBodyVar).
 * @param pb Body point; z≈0 is clamped.
 * @param range_inc Depth noise (m).
 * @param degree_inc Beam angle noise (deg).
 * @param var Output 3×3 covariance.
 */
void CalcBodyVar(Eigen::Vector3d pb, float range_inc, float degree_inc,
                 Eigen::Matrix3d* var);

/**
 * @brief Init PointVar list from cloud in extrinsic frame (var_init).
 */
void InitPointVars(const ImuState& extrin, const sensor::PointCloudXYZI& cloud,
                   double dept_err, double beam_err, PointVarList* out);

/**
 * @brief Propagate body vars to world and fill world points (pvec_update).
 */
void UpdateWorldVars(const ImuState& x, PointVarList* pvec,
                     VectorList<3>* pwld);

}  // namespace vxslam
}  // namespace argus

#endif

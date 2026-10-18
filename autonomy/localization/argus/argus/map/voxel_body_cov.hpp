/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file voxel_body_cov.hpp
 * @brief Body-frame LiDAR point covariance helper for LIO.
 */

#ifndef ARGUS_MAP_VOXEL_BODY_COV_HPP_
#define ARGUS_MAP_VOXEL_BODY_COV_HPP_

#include <Eigen/Dense>

namespace argus {
namespace map {

/**
 * @brief Approximate body-frame covariance from range / beam angular error.
 * @param[in,out] pb Point in body (z≈0 is nudged).
 * @param range_inc Range noise scale.
 * @param degree_inc Beam angle noise (deg).
 * @param[out] cov 3×3 covariance.
 */
void CalcBodyCov(Eigen::Vector3d& pb, float range_inc, float degree_inc,
                 Eigen::Matrix3d& cov);

/** @brief Legacy name. */
inline void calcBodyCov(Eigen::Vector3d& pb, const float range_inc,
                        const float degree_inc, Eigen::Matrix3d& cov) {
    CalcBodyCov(pb, range_inc, degree_inc, cov);
}

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_VOXEL_BODY_COV_HPP_

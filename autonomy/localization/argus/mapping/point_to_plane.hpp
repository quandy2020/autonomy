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
 * @file point_to_plane.hpp
 * @brief Point-to-plane residual association for LIO ESIKF.
 */

#ifndef ARGUS_MAP_POINT_TO_PLANE_HPP_
#define ARGUS_MAP_POINT_TO_PLANE_HPP_

#include <Eigen/Dense>

namespace argus {
namespace map {

/** @brief One point–plane correspondence used in the LIO update. */
struct PointToPlane {
    Eigen::Vector3d point_b_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d point_w_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d center_ = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 6, 6> plane_var_ =
        Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix3d body_cov_ = Eigen::Matrix3d::Zero();
    int layer_ = 0;
    double d_ = 0;
    double eigen_value_ = 0;
    bool is_valid_ = false;
    float dis_to_plane_ = 0;
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_POINT_TO_PLANE_HPP_

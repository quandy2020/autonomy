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
 * @file voxel_plane.hpp
 * @brief Fitted plane parameters inside a voxel octree node.
 */

#ifndef ARGUS_MAP_VOXEL_PLANE_HPP_
#define ARGUS_MAP_VOXEL_PLANE_HPP_

#include <Eigen/Dense>

namespace argus {
namespace map {

/** @brief Plane fit result stored on a VoxelOctoTree leaf / node. */
struct VoxelPlane {
    Eigen::Vector3d center_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d y_normal_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d x_normal_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covariance_ = Eigen::Matrix3d::Zero();
    Eigen::Matrix<double, 6, 6> plane_var_ =
        Eigen::Matrix<double, 6, 6>::Zero();
    float radius_ = 0;
    float min_eigen_value_ = 1;
    float mid_eigen_value_ = 1;
    float max_eigen_value_ = 1;
    float d_ = 0;
    int points_size_ = 0;
    bool is_plane_ = false;
    bool is_init_ = false;
    int id_ = 0;
    bool is_update_ = false;
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_VOXEL_PLANE_HPP_

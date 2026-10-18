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
 * @file lio_frame_buffers.hpp
 * @brief Per-scan LIO clouds and ESIKF scratch buffers.
 */

#ifndef ARGUS_MAP_LIO_FRAME_BUFFERS_HPP_
#define ARGUS_MAP_LIO_FRAME_BUFFERS_HPP_

#include <vector>

#include <Eigen/Dense>

#include "argus/frontend/types.hpp"
#include "argus/map/point_to_plane.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace map {

using frontend::PointWithVar;
using sensor::PointCloud;
using sensor::PointCloudXYZI;
using sensor::PointXYZI;

/**
 * @brief Per-scan LIO clouds + ESIKF scratch (capacity reused across frames).
 *
 * Owned by `LivoOdometry::lio_frame_` (production) or
 * `VoxelMapManager` internal storage (standalone tests). Access via
 * `VoxelMapManager::frame()` after optional `BindFrameBuffers`.
 */
struct LioFrameBuffers {
    PointCloudXYZI::Ptr feats_undistort;  /**< Undistorted scan (body). */
    PointCloudXYZI::Ptr feats_down_body;  /**< Downsampled body features. */
    PointCloudXYZI::Ptr feats_down_world; /**< Downsampled world features. */
    PointCloud<PointXYZI>::Ptr world_lidar; /**< TransformLidar output buffer. */
    int feats_down_size = 0;
    int effect_feat_num = 0;
    std::vector<Eigen::Matrix3d> cross_mat_list;
    std::vector<Eigen::Matrix3d> body_cov_list;
    std::vector<PointWithVar> pv_list;
    std::vector<PointToPlane> ptpl_list;
    std::vector<PointToPlane> residual_ptpl_scratch;
    std::vector<bool> residual_useful_scratch;
    /** @brief ESIKF H / measurement scratch (resized each iteration). */
    Eigen::MatrixXd Hsub;
    Eigen::MatrixXd Hsub_T_R_inv;
    Eigen::VectorXd R_inv;
    Eigen::VectorXd meas_vec;

    /** @brief Allocate empty shared clouds if null. */
    void EnsureDefaultClouds();
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_LIO_FRAME_BUFFERS_HPP_

/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file local_esdf.hpp
 * @brief Build a robot-frame ESDF from depth using map/esdf (SanD critic).
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "autonomy/control/controller/policy_controller/types.hpp"
#include "autonomy/control/proto/policy_controller.pb.h"
#include "autonomy/map/esdf/esdf_map.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

/**
 * @brief Map geometry for @ref map::esdf::EsdfMap from SanD-Planner options.
 *
 * Origin and size are in the robot frame (x forward). Resolution, ray length
 * and occupancy fusion follow @p options; two occupied hits are still required
 * before a voxel is occupied (EsdfMap default probabilities).
 *
 * @param[in] options Effective SanD-Planner sidecar (esdf_* filled).
 */
map::esdf::Options MakeLocalEsdfOptions(const proto::SandPlanner& options);

/**
 * @brief Unproject the latest depth image into robot-frame hit points.
 *
 * 32FC1 is metres, 16UC1 is millimetres. @c depth_optical_frame maps a
 * z-forward camera into x-forward robot coordinates.
 *
 * @param[in] snapshot Uses depth_history.back(), else image.
 * @param[in] options Intrinsics, max_depth, pixel_stride.
 * @param[out] error Optional decode failure.
 * @return Finite hit points; empty when there is no usable depth.
 */
std::vector<Eigen::Vector3d> UnprojectDepthToRobot(
    const ObservationSnapshot& snapshot, const proto::SandPlanner& options,
    std::string* error = nullptr);

/**
 * @brief Integrate the depth cloud twice, then UpdateOccupancy + UpdateESDF.
 *
 * The map lives in the robot frame with the sensor at the origin. Two
 * IntegrateCloud / UpdateOccupancy passes match FIESTA's two-hit occupancy.
 *
 * @param[in] snapshot Latest depth.
 * @param[in] options Geometry and unprojection knobs.
 * @param[out] error Optional failure (decode only; empty cloud still returns a
 *                   map with no occupied cells).
 * @return Heap map, or nullptr only when @p error is a hard decode failure
 *         and the snapshot claimed a depth image that could not be parsed.
 */
std::unique_ptr<map::esdf::EsdfMap> BuildLocalEsdfFromDepth(
    const ObservationSnapshot& snapshot, const proto::SandPlanner& options,
    std::string* error = nullptr);

/**
 * @brief Measured clearance in metres.
 *
 * Returns NaN when @p position is outside the map or the cell was never given
 * a finite distance (empty map). Callers must skip those samples so kInfinity
 * does not dominate the critic.
 *
 * @param[in] map Local ESDF from @ref BuildLocalEsdfFromDepth.
 * @param[in] position Robot-frame query.
 */
double QueryLocalClearance(const map::esdf::EsdfMap& map,
                           const Eigen::Vector3d& position);

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

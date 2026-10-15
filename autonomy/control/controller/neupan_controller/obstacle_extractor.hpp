/*
 * Copyright 2026 The Openbot Authors (duyongquan)
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
 * @file obstacle_extractor.hpp
 * @brief LaserScan → Eigen obstacle points (base frame) for NeupanController.
 */

#pragma once

#include <Eigen/Core>

#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include "autonomy/control/controller/neupan_controller/types.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {

/**
 * @brief Extract strided obstacle points into a reused Eigen matrix.
 *
 * Writes shape [2, N] into @p points_out (resized). When @p max_point_count >
 * 0, beam indices are strided so N <= max_point_count.
 *
 * @param[in] laser_scan Latest scan.
 * @param[in] extrinsic Laser pose / range limits in the base frame.
 * @param[in] max_point_count Cap on columns; <= 0 keeps all valid returns.
 * @param[in,out] points_out Output matrix (capacity reused via resize).
 * @return Number of valid points written (equals points_out.cols()).
 */
int ExtractObstaclePointsFromLaserScan(
    const automsgs::msgs::sensor_msgs::LaserScan& laser_scan,
    const LaserExtrinsic& extrinsic, int max_point_count,
    Eigen::Matrix2Xd* points_out);

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

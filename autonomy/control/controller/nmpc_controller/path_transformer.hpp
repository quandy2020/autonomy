/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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
 * @file path_transformer.hpp
 * @brief Global-plan TF transform and pruning for NMPC
 */

#pragma once

#include <memory>
#include <string>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include "autonomy/control/proto/nmpc_controller.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {

/**
 * @class nmpc_controller::PathTransformer
 * @brief Transforms and prunes a global plan into the costmap frame
 *
 * Similar in role to mppi::PathHandler: finds the closest waypoint, extracts a
 * forward window, and transforms poses into the costmap global frame.
 */
class PathTransformer {
 public:
    /**
     * @brief Initialize path transformer on bringup
     * @param tf_buffer TF buffer for transformations
     * @param costmap_wrapper Costmap2DWrapper object of environment
     * @param options NMPC path-handling options
     */
    void Configure(std::shared_ptr<transform::Buffer> tf_buffer,
                   std::shared_ptr<map::costmap_2d::Costmap2DWrapper>
                       costmap_wrapper,
                   const proto::NMPCControllerOptions& options);

    /**
     * @brief Set new global reference path
     * @param plan Global path to track
     */
    void SetPlan(const automsgs::msgs::nav_msgs::Path& plan);

    /**
     * @brief Transform and prune the plan around the robot
     * @param robot_pose Current robot pose
     * @return Local path segment in the costmap frame
     */
    automsgs::msgs::nav_msgs::Path Transform(
        const automsgs::msgs::geometry_msgs::PoseStamped& robot_pose) const;

 private:
    // TF buffer for transforming path poses into the costmap frame
    std::shared_ptr<transform::Buffer> tf_buffer_;
    // Costmap used to bound the extracted local path window
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
    // Path-handling parameters (prune distance, transform tolerance, etc.)
    proto::NMPCControllerOptions options_;
    // Global plan stored by SetPlan before transformation
    automsgs::msgs::nav_msgs::Path global_plan_;
};

}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

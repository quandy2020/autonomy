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
 * @file controller.hpp
 * @brief NeupanController plugin: LaserScan + NeuPanPlanner.
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <automsgs/msgs/geometry_msgs/pose2d.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <Eigen/Core>

#include "autonomy/control/common/controller_interface.hpp"
#include "autonomy/control/controller/neupan_controller/obstacle_extractor.hpp"
#include "autonomy/control/controller/neupan_controller/planner.hpp"
#include "autonomy/control/controller/neupan_controller/types.hpp"
#include "autonomy/control/proto/neupan_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {

/**
 * @class NeupanController
 * @brief Map-free local controller: DUNE (ONNX) + NRMP (DAQP) + PAN.
 *
 * @c SetPlan feeds @ref NeuPanPlanner::SetInitialPath. Each tick extracts
 * laser points in the base frame, transforms them to world, and runs
 * @ref NeuPanPlanner::Forward.
 */
class NeupanController : public common::ControllerInterface
{
public:
    NeupanController() = default;
    ~NeupanController() override = default;

    void Configure(
        const proto::ControllerOptions& options, std::string name,
        std::shared_ptr<transform::Buffer> tf_buffer,
        std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper);

    void Cleanup();
    void Activate();
    void Deactivate();

    uint32 ComputeVelocityCommands(
        const automsgs::msgs::geometry_msgs::PoseStamped& pose,
        const automsgs::msgs::geometry_msgs::TwistStamped& velocity,
        automsgs::msgs::geometry_msgs::TwistStamped& cmd_vel,
        common::GoalChecker* goal_checker, std::string& message) override;

    bool IsGoalReached(double distance_tolerance,
                       double angle_tolerance) override;
    void SetPlan(const automsgs::msgs::nav_msgs::Path& plan) override;
    void SetSpeedLimit(const double& speed_limit,
                       const bool& percentage) override;
    void Reset() override;

    void SetLaserScan(const automsgs::msgs::sensor_msgs::LaserScan& laser_scan);

private:
    bool EnsureInitialized(std::string* error);
    Mat2X CreateVertices(std::string* error) const;
    void CacheLaserExtrinsic();

    proto::NeupanControllerOptions options_neupan_;
    std::unique_ptr<NeuPanPlanner> planner_;
    LaserExtrinsic laser_extrinsic_;
    bool omni_{false};

    std::vector<InitialPath::PathPoint> path_points_;
    std::mutex planner_mutex_;

    std::shared_ptr<const automsgs::msgs::sensor_msgs::LaserScan> laser_scan_;
    std::mutex laser_scan_mutex_;

    Eigen::Matrix2Xd obstacle_points_buffer_;

    double speed_limit_scale_{1.0};
    double goal_position_tolerance_{0.25};
    int max_encoder_point_count_{100};
    automsgs::msgs::geometry_msgs::Pose2D last_pose_;
    bool has_pose_{false};
};

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

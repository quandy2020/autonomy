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

#pragma once

#include <memory>
#include <string>

#include "autonomy/control/common/controller_interface.hpp"
#include "autonomy/control/controller/graceful_controller/path_handler.hpp"
#include "autonomy/control/controller/nmpc_controller/models/kinematic_pose.hpp"
#include "autonomy/control/controller/nmpc_controller/optimization/mpc_solver.hpp"
#include "autonomy/control/controller/nmpc_controller/tracking/path_reference.hpp"
#include "autonomy/control/proto/nmpc_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {

/**
 * @brief NMPC local controller (Ipopt + planar kinematic models).
 */
class NmpcController : public common::ControllerInterface
{
public:
    NmpcController() = default;
    ~NmpcController() override = default;

    void Configure(const proto::ControllerOptions& options, std::string name,
                   std::shared_ptr<transform::Buffer> tf,
                   std::shared_ptr<map::costmap_2d::Costmap2DWrapper>
                       costmap_wrapper) override;

    void Cleanup() override;
    void Activate() override;
    void Deactivate() override;

    uint32 ComputeVelocityCommands(
        const commsgs::geometry_msgs::PoseStamped& pose,
        const commsgs::geometry_msgs::TwistStamped& velocity,
        commsgs::geometry_msgs::TwistStamped& cmd_vel,
        common::GoalChecker* goal_checker, std::string& message) override;

    bool IsGoalReached(double dist_tolerance, double angle_tolerance) override;

    void SetPlan(const commsgs::planning_msgs::Path& path) override;

    void SetSpeedLimit(const double& speed_limit,
                       const bool& percentage) override;

private:
    nmpc::models::Pose2D PoseToState(
        const commsgs::geometry_msgs::PoseStamped& pose) const;

    void ApplySpeedLimit(nmpc::models::BodyTwist& twist) const;

    double ComputeHolonomicYawRate(
        const nmpc::models::Pose2D& current_state,
        const std::vector<nmpc::models::Pose2D>& references) const;

    std::string plugin_name_;
    proto::NmpcControllerOptions nmpc_options_;
    std::shared_ptr<transform::Buffer> tf_buffer_;
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
    std::unique_ptr<PathHandler> path_handler_;
    std::unique_ptr<nmpc::mpc_opt::MpcSolver> mpc_solver_;
    std::unique_ptr<nmpc::tracking::PathReference> path_reference_;

    double speed_limit_{0.0};
    bool speed_limit_percentage_{false};
    bool configured_{false};
};

}  // namespace controller
}  // namespace control
}  // namespace autonomy

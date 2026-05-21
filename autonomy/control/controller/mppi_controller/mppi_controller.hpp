/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/control/common/controller_interface.hpp"
#include "autonomy/control/controller/graceful_controller/path_handler.hpp"
#include "autonomy/control/controller/mppi_controller/optimizer.hpp"
#include "autonomy/control/proto/mppi_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {

/**
 * @brief MPPI local trajectory controller (non-ROS, autonomy commsgs/costmap).
 */
class MppiController : public common::ControllerInterface
{
public:
    MppiController() = default;
    ~MppiController() override = default;

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
    std::string plugin_name_;
    proto::MPPIControllerOptions mppi_options_;
    std::shared_ptr<transform::Buffer> tf_buffer_;
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
    std::unique_ptr<PathHandler> path_handler_;
    std::unique_ptr<mppi::Optimizer> optimizer_;
    double controller_frequency_{20.0};
    double max_robot_pose_search_dist_{10.0};
    bool configured_{false};
};

}  // namespace controller
}  // namespace control
}  // namespace autonomy

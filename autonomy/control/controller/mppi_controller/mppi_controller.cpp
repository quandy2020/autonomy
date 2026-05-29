/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/mppi_controller/mppi_controller.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/common/math/math.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/control/proto/controller_options.pb.h"
#include "autonomy/map/costmap_2d/filters/filter_values.hpp"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace control {
namespace controller {

namespace {
using proto::ControllerResultCode;
}  // namespace

void MppiController::Configure(
    const proto::ControllerOptions& options, std::string name,
    std::shared_ptr<transform::Buffer> tf,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
    costmap_wrapper_ = std::move(costmap_wrapper);
    tf_buffer_ = std::move(tf);
    plugin_name_ = std::move(name);

    if (options.has_mppi_controller_options()) {
        mppi_options_ = options.mppi_controller_options();
    }

    controller_frequency_ =
        options.controller_frequency() > 0.0 ? options.controller_frequency()
                                             : 20.0;

    const double transform_tolerance = 0.1;
    max_robot_pose_search_dist_ = 10.0;

    path_handler_ = std::make_unique<PathHandler>(
        transform_tolerance, tf_buffer_, costmap_wrapper_);
    optimizer_ = std::make_unique<mppi::Optimizer>();
    optimizer_->initialize(mppi_options_, costmap_wrapper_,
                           controller_frequency_);

    configured_ = true;
    AINFO << "Loaded controller: " << plugin_name_;
}

void MppiController::Cleanup() {
    if (optimizer_) {
        optimizer_->shutdown();
    }
    optimizer_.reset();
    path_handler_.reset();
    configured_ = false;
}

void MppiController::Activate() {
    if (optimizer_) {
        optimizer_->reset(false);
    }
}

void MppiController::Deactivate() {
    AINFO << "Deactivating controller: " << plugin_name_;
}

void MppiController::SetPlan(const commsgs::planning_msgs::Path& path) {
    if (path_handler_) {
        path_handler_->SetPlan(path);
    }
    if (optimizer_) {
        optimizer_->reset(false);
    }
}

void MppiController::SetSpeedLimit(const double& speed_limit,
                                   const bool& percentage) {
    if (optimizer_) {
        optimizer_->setSpeedLimit(speed_limit, percentage);
    }
}

uint32 MppiController::ComputeVelocityCommands(
    const commsgs::geometry_msgs::PoseStamped& pose,
    const commsgs::geometry_msgs::TwistStamped& velocity,
    commsgs::geometry_msgs::TwistStamped& cmd_vel,
    common::GoalChecker* goal_checker, std::string& message) {
    cmd_vel.twist.linear.x = 0.0f;
    cmd_vel.twist.linear.y = 0.0f;
    cmd_vel.twist.linear.z = 0.0f;
    cmd_vel.twist.angular.x = 0.0f;
    cmd_vel.twist.angular.y = 0.0f;
    cmd_vel.twist.angular.z = 0.0f;

    if (!configured_ || !path_handler_ || !optimizer_ || !costmap_wrapper_ ||
        !tf_buffer_) {
        message = "MppiController not configured";
        return ControllerResultCode::CONTROLLER_RESULT_NOT_INITIALIZED;
    }

    try {
        auto local_plan =
            path_handler_->TransformGlobalPlan(pose, max_robot_pose_search_dist_);
        if (local_plan.poses.empty()) {
            message = "Empty transformed plan";
            return ControllerResultCode::CONTROLLER_RESULT_INVALID_PATH;
        }

        commsgs::geometry_msgs::Pose goal;
        goal = local_plan.poses.back().pose;

        cmd_vel = optimizer_->evalControl(pose, velocity.twist, local_plan, goal,
                                          goal_checker);
        message.clear();
        return ControllerResultCode::CONTROLLER_RESULT_SUCCESS;
    } catch (const common::ControllerException& ex) {
        message = ex.what();
        AWARN << "MppiController: " << message;
        return ControllerResultCode::CONTROLLER_RESULT_NO_VALID_CMD;
    } catch (const std::exception& ex) {
        message = ex.what();
        AWARN << "MppiController exception: " << message;
        return ControllerResultCode::CONTROLLER_RESULT_INTERNAL_ERROR;
    }
}

bool MppiController::IsGoalReached(double dist_tolerance,
                                   double angle_tolerance) {
    if (!path_handler_ || !costmap_wrapper_ || !tf_buffer_) {
        return false;
    }
    const auto& plan = path_handler_->GetPlan();
    if (plan.poses.empty()) {
        return true;
    }

    try {
        auto goal = tf_buffer_->transform(
            plan.poses.back(), costmap_wrapper_->getBaseFrameID(), 0.1f);
        commsgs::geometry_msgs::PoseStamped robot_pose;
        robot_pose.header.frame_id = costmap_wrapper_->getBaseFrameID();
        robot_pose.pose.orientation.w = 1.0f;
        robot_pose = tf_buffer_->transform(
            robot_pose, costmap_wrapper_->getBaseFrameID(), 0.1f);

        const double dx = static_cast<double>(goal.pose.position.x) -
                          static_cast<double>(robot_pose.pose.position.x);
        const double dy = static_cast<double>(goal.pose.position.y) -
                          static_cast<double>(robot_pose.pose.position.y);
        const double dist = std::hypot(dx, dy);
        const double yaw_err = std::abs(
            ::autonomy::common::NormalizeAngleDifference(
                transform::tf2::getYaw(goal.pose.orientation) -
                transform::tf2::getYaw(robot_pose.pose.orientation)));
        return dist <= dist_tolerance && yaw_err <= angle_tolerance;
    } catch (const std::exception& ex) {
        AWARN << "MppiController IsGoalReached TF error: " << ex.what();
        return false;
    }
}

}  // namespace controller
}  // namespace control
}  // namespace autonomy

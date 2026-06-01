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

#include "autonomy/control/controller/tdmpc_controller/tdmpc_controller.hpp"

#include <cmath>

#include "autonomy/common/logging.hpp"
#include "autonomy/common/math/math.hpp"
#include "autonomy/control/proto/controller_options.pb.h"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace control {
namespace controller {

namespace {

using proto::ControllerResultCode;

}  // namespace

void TdmpcController::Configure(
    const proto::ControllerOptions& options, std::string name,
    std::shared_ptr<transform::Buffer> tf,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
    costmap_wrapper_ = std::move(costmap_wrapper);
    tf_buffer_ = std::move(tf);
    plugin_name_ = std::move(name);

    if (options.has_tdmpc_controller_options()) {
        tdmpc_options_ = options.tdmpc_controller_options();
    }

    const double transform_tolerance =
        tdmpc_options_.transform_tolerance() > 0.0
            ? tdmpc_options_.transform_tolerance()
            : 0.1;

    path_handler_ = std::make_unique<PathHandler>(
        transform_tolerance, tf_buffer_, costmap_wrapper_);
    mpc_solver_ = std::make_unique<tdmpc::mpc_opt::TdmpcSolver>(
        tdmpc_options_, costmap_wrapper_.get());

    configured_ = true;
}

void TdmpcController::Cleanup() {
    path_handler_.reset();
    mpc_solver_.reset();
    configured_ = false;
}

void TdmpcController::Activate() {}

void TdmpcController::Deactivate() {
    AINFO << "Deactivating controller: " << plugin_name_;
}

nmpc::models::Pose2D TdmpcController::PoseToState(
    const commsgs::geometry_msgs::PoseStamped& pose) const {
    nmpc::models::Pose2D state;
    state.x = static_cast<double>(pose.pose.position.x);
    state.y = static_cast<double>(pose.pose.position.y);
    state.theta = transform::tf2::getYaw(pose.pose.orientation);
    return state;
}

void TdmpcController::SetPlan(const commsgs::planning_msgs::Path& path) {
    if (path_handler_) {
        path_handler_->SetPlan(path);
    }
}

void TdmpcController::SetSpeedLimit(const double& speed_limit,
                                    const bool& percentage) {
    speed_limit_ = speed_limit;
    speed_limit_percentage_ = percentage;
}

void TdmpcController::ApplySpeedLimit(nmpc::models::BodyTwist& twist) const {
    if (speed_limit_ <= 0.0) {
        return;
    }
    if (speed_limit_percentage_) {
        const double v_max = tdmpc_options_.v_max() > tdmpc_options_.v_min()
                                 ? tdmpc_options_.v_max()
                                 : 0.5;
        const double scale = speed_limit_ / 100.0;
        twist.v = std::min(twist.v, v_max * scale);
        twist.vy = std::min(twist.vy, v_max * scale);
        return;
    }
    const double speed = twist.LinearSpeed();
    if (speed > speed_limit_ && speed > 1e-9) {
        const double scale = speed_limit_ / speed;
        twist.v *= scale;
        twist.vy *= scale;
    }
}

double TdmpcController::ComputeHolonomicYawRate(
    const nmpc::models::Pose2D& current_state,
    const tdmpc::tracking::ContouringHorizon& horizon) const {
    if (horizon.poses.size() < 2) {
        return 0.0;
    }
    const double dt =
        tdmpc_options_.dt() > 0.0 ? tdmpc_options_.dt() : 0.1;
    const double ref_omega =
        ::autonomy::common::NormalizeAngleDifference(horizon.poses[1].theta -
                                         horizon.poses[0].theta) /
        dt;
    const double kp =
        tdmpc_options_.yaw_rate_kp() > 0.0 ? tdmpc_options_.yaw_rate_kp() : 1.5;
    const double yaw_err = ::autonomy::common::NormalizeAngleDifference(
        horizon.poses[1].theta - current_state.theta);
    double omega = ref_omega + kp * yaw_err;
    const double omega_lo = std::min(tdmpc_options_.omega_min(),
                                     tdmpc_options_.omega_max());
    const double omega_hi = std::max(tdmpc_options_.omega_min(),
                                     tdmpc_options_.omega_max());
    return std::clamp(omega, omega_lo, omega_hi);
}

uint32 TdmpcController::ComputeVelocityCommands(
    const commsgs::geometry_msgs::PoseStamped& pose,
    const commsgs::geometry_msgs::TwistStamped& velocity,
    commsgs::geometry_msgs::TwistStamped& cmd_vel,
    common::GoalChecker* /*goal_checker*/, std::string& message) {
    cmd_vel.twist.linear.x = 0.0f;
    cmd_vel.twist.linear.y = 0.0f;
    cmd_vel.twist.linear.z = 0.0f;
    cmd_vel.twist.angular.x = 0.0f;
    cmd_vel.twist.angular.y = 0.0f;
    cmd_vel.twist.angular.z = 0.0f;
    (void)velocity;

    if (!configured_ || !path_handler_ || !mpc_solver_) {
        message = "TdmpcController not configured";
        return ControllerResultCode::CONTROLLER_RESULT_NOT_INITIALIZED;
    }

    const double max_search =
        tdmpc_options_.max_robot_pose_search_dist() > 0.0
            ? tdmpc_options_.max_robot_pose_search_dist()
            : 3.0;
    auto local_plan = path_handler_->TransformGlobalPlan(pose, max_search);
    if (local_plan.poses.empty()) {
        message = "Empty transformed plan";
        return ControllerResultCode::CONTROLLER_RESULT_INVALID_PATH;
    }

    if (!path_spline_.BuildFromPath(local_plan)) {
        message = "Failed to build path spline";
        return ControllerResultCode::CONTROLLER_RESULT_INVALID_PATH;
    }

    const auto current_state = PoseToState(pose);
    nmpc::models::BodyTwist control;
    if (!mpc_solver_->Solve(current_state, path_spline_, control)) {
        message = "T-MPC solve failed";
        return ControllerResultCode::CONTROLLER_RESULT_NO_VALID_CMD;
    }

    if (mpc_solver_->kinematicModel().IsHolonomicQuadruped()) {
        control.omega =
            ComputeHolonomicYawRate(current_state, mpc_solver_->lastHorizon());
    }

    ApplySpeedLimit(control);

    message.clear();
    if (mpc_solver_->usedFallback()) {
        message = "T-MPC fallback (previous solution)";
    } else {
        message = "topology_id=" + std::to_string(mpc_solver_->selectedTopologyId());
    }

    cmd_vel.twist.linear.x = static_cast<float>(control.v);
    cmd_vel.twist.linear.y = static_cast<float>(control.vy);
    cmd_vel.twist.angular.z = static_cast<float>(control.omega);
    cmd_vel.header.frame_id = costmap_wrapper_->getBaseFrameID();
    cmd_vel.header.stamp = pose.header.stamp;

    return ControllerResultCode::CONTROLLER_RESULT_SUCCESS;
}

bool TdmpcController::IsGoalReached(double dist_tolerance,
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
            plan.poses.back(), costmap_wrapper_->getBaseFrameID(),
            static_cast<float>(tdmpc_options_.transform_tolerance()));
        commsgs::geometry_msgs::PoseStamped robot_pose;
        robot_pose.header.frame_id = costmap_wrapper_->getBaseFrameID();
        robot_pose.pose.orientation.w = 1.0f;
        robot_pose = tf_buffer_->transform(
            robot_pose, costmap_wrapper_->getBaseFrameID(),
            static_cast<float>(tdmpc_options_.transform_tolerance()));

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
        AWARN << "TdmpcController IsGoalReached TF error: " << ex.what();
        return false;
    }
}

}  // namespace controller
}  // namespace control
}  // namespace autonomy

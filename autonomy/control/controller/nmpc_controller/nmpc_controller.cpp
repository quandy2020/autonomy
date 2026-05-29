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

#include "autonomy/control/controller/nmpc_controller/nmpc_controller.hpp"

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

void NmpcController::Configure(
    const proto::ControllerOptions& options, std::string name,
    std::shared_ptr<transform::Buffer> tf,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
    costmap_wrapper_ = std::move(costmap_wrapper);
    tf_buffer_ = std::move(tf);
    plugin_name_ = std::move(name);

    if (options.has_nmpc_controller_options()) {
        nmpc_options_ = options.nmpc_controller_options();
    }

    const double transform_tolerance =
        nmpc_options_.transform_tolerance() > 0.0
            ? nmpc_options_.transform_tolerance()
            : 0.1;

    const int horizon =
        nmpc_options_.horizon() > 0 ? nmpc_options_.horizon() : 10;
    const double dt =
        nmpc_options_.dt() > 0.0 ? nmpc_options_.dt() : 0.1;
    const double ref_v =
        nmpc_options_.reference_velocity() > 0.0
            ? nmpc_options_.reference_velocity()
            : 0.3;
    const double slowdown =
        nmpc_options_.slowdown_radius() > 0.0
            ? nmpc_options_.slowdown_radius()
            : 0.0;

    path_handler_ = std::make_unique<PathHandler>(
        transform_tolerance, tf_buffer_, costmap_wrapper_);
    mpc_solver_ =
        std::make_unique<nmpc::mpc_opt::MpcSolver>(nmpc_options_);
    path_reference_ = std::make_unique<nmpc::tracking::PathReference>(
        horizon, dt, ref_v, slowdown);

    configured_ = true;
    AINFO << "Loaded controller: " << plugin_name_;
}

void NmpcController::Cleanup() {
    path_handler_.reset();
    mpc_solver_.reset();
    path_reference_.reset();
    configured_ = false;
}

void NmpcController::Activate() {}

void NmpcController::Deactivate() {
    AINFO << "Deactivating controller: " << plugin_name_;
}

nmpc::models::Pose2D NmpcController::PoseToState(
    const commsgs::geometry_msgs::PoseStamped& pose) const {
    nmpc::models::Pose2D state;
    state.x = static_cast<double>(pose.pose.position.x);
    state.y = static_cast<double>(pose.pose.position.y);
    state.theta = transform::tf2::getYaw(pose.pose.orientation);
    return state;
}

void NmpcController::SetPlan(const commsgs::planning_msgs::Path& path) {
    if (path_handler_) {
        path_handler_->SetPlan(path);
    }
}

void NmpcController::SetSpeedLimit(const double& speed_limit,
                                   const bool& percentage) {
    speed_limit_ = speed_limit;
    speed_limit_percentage_ = percentage;
}

void NmpcController::ApplySpeedLimit(nmpc::models::BodyTwist& twist) const {
    if (speed_limit_ <= 0.0) {
        return;
    }
    if (speed_limit_percentage_) {
        const double v_max = nmpc_options_.v_max() > nmpc_options_.v_min()
                                 ? nmpc_options_.v_max()
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

double NmpcController::ComputeHolonomicYawRate(
    const nmpc::models::Pose2D& current_state,
    const std::vector<nmpc::models::Pose2D>& references) const {
    if (references.size() < 2) {
        return 0.0;
    }
    const double dt =
        nmpc_options_.dt() > 0.0 ? nmpc_options_.dt() : 0.1;
    const double ref_omega =
        ::autonomy::common::NormalizeAngleDifference(references[1].theta -
                                         references[0].theta) /
        dt;
    const double kp =
        nmpc_options_.yaw_rate_kp() > 0.0 ? nmpc_options_.yaw_rate_kp() : 1.5;
    const double yaw_err = ::autonomy::common::NormalizeAngleDifference(
        references[1].theta - current_state.theta);
    double omega = ref_omega + kp * yaw_err;

    const double omega_lo = std::min(nmpc_options_.omega_min(),
                                     nmpc_options_.omega_max());
    const double omega_hi = std::max(nmpc_options_.omega_min(),
                                     nmpc_options_.omega_max());
    return std::clamp(omega, omega_lo, omega_hi);
}

uint32 NmpcController::ComputeVelocityCommands(
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

    if (!configured_ || !path_handler_ || !mpc_solver_ || !path_reference_) {
        message = "NmpcController not configured";
        return ControllerResultCode::CONTROLLER_RESULT_NOT_INITIALIZED;
    }

    if (!costmap_wrapper_ || !tf_buffer_) {
        message = "NmpcController missing costmap or TF";
        return ControllerResultCode::CONTROLLER_RESULT_NOT_INITIALIZED;
    }

    const double max_search =
        nmpc_options_.max_robot_pose_search_dist() > 0.0
            ? nmpc_options_.max_robot_pose_search_dist()
            : 3.0;
    auto local_plan = path_handler_->TransformGlobalPlan(pose, max_search);
    if (local_plan.poses.empty()) {
        message = "Empty transformed plan";
        return ControllerResultCode::CONTROLLER_RESULT_INVALID_PATH;
    }

    const auto current_state = PoseToState(pose);
    if (!path_reference_->BuildFromPath(local_plan, current_state)) {
        message = "Failed to build NMPC path references";
        return ControllerResultCode::CONTROLLER_RESULT_INVALID_PATH;
    }

    const auto& references = path_reference_->references();
    nmpc::models::BodyTwist control;
    if (!mpc_solver_->Solve(current_state, references, control)) {
        message = "NMPC solve failed";
        return ControllerResultCode::CONTROLLER_RESULT_NO_VALID_CMD;
    }

    if (mpc_solver_->kinematicModel().IsHolonomicQuadruped()) {
        control.omega = ComputeHolonomicYawRate(current_state, references);
    }

    ApplySpeedLimit(control);

    if (mpc_solver_->usedFallback()) {
        message = "NMPC using fallback control (previous solution)";
    } else {
        message.clear();
    }

    cmd_vel.twist.linear.x = static_cast<float>(control.v);
    cmd_vel.twist.linear.y = static_cast<float>(control.vy);
    cmd_vel.twist.angular.z = static_cast<float>(control.omega);
    cmd_vel.header.frame_id = costmap_wrapper_->getBaseFrameID();
    cmd_vel.header.stamp = pose.header.stamp;

    return ControllerResultCode::CONTROLLER_RESULT_SUCCESS;
}

bool NmpcController::IsGoalReached(double dist_tolerance,
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
            static_cast<float>(nmpc_options_.transform_tolerance()));
        commsgs::geometry_msgs::PoseStamped robot_pose;
        robot_pose.header.frame_id = costmap_wrapper_->getBaseFrameID();
        robot_pose.pose.orientation.w = 1.0f;
        robot_pose = tf_buffer_->transform(
            robot_pose, costmap_wrapper_->getBaseFrameID(),
            static_cast<float>(nmpc_options_.transform_tolerance()));

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
        AWARN << "NmpcController IsGoalReached TF error: " << ex.what();
        return false;
    }
}

}  // namespace controller
}  // namespace control
}  // namespace autonomy

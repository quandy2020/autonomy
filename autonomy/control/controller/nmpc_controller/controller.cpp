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
 * @file controller.cpp
 * @brief Implementation of nmpc_controller::NMPCController
 */

#include "autonomy/control/controller/nmpc_controller/controller.hpp"

#include <cmath>
#include <limits>

#include "autolink/class_loader/class_loader_register_macro.hpp"
#include "autolink/common/log.hpp"
#include "autonomy/control/controller/nmpc_controller/parameter_defaults.hpp"
#include "autonomy/control/proto/controller_options.pb.h"
#include "autonomy/map/costmap_2d/filters/filter_values.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {

namespace {

/**
 * @brief Extract yaw from a geometry_msgs quaternion (Z-up, planar robot).
 */
double YawFromPose(const automsgs::msgs::geometry_msgs::Pose& pose) {
    const auto& q = pose.orientation();
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

/**
 * @brief Publish zero linear/angular velocity.
 */
void SetZeroVelocity(automsgs::msgs::geometry_msgs::TwistStamped& cmd_vel) {
    cmd_vel.mutable_twist()->mutable_linear()->set_x(0.0);
    cmd_vel.mutable_twist()->mutable_linear()->set_y(0.0);
    cmd_vel.mutable_twist()->mutable_linear()->set_z(0.0);
    cmd_vel.mutable_twist()->mutable_angular()->set_x(0.0);
    cmd_vel.mutable_twist()->mutable_angular()->set_y(0.0);
    cmd_vel.mutable_twist()->mutable_angular()->set_z(0.0);
}

/**
 * @brief Map NMPC command [v, omega] to TwistStamped.
 */
void ApplyCommand(const NmpcOptimizer::SolveResult& result,
                  automsgs::msgs::geometry_msgs::TwistStamped& cmd_vel) {
    cmd_vel.mutable_twist()->mutable_linear()->set_x(result.cmd(0));
    cmd_vel.mutable_twist()->mutable_linear()->set_y(0.0);
    cmd_vel.mutable_twist()->mutable_linear()->set_z(0.0);
    cmd_vel.mutable_twist()->mutable_angular()->set_x(0.0);
    cmd_vel.mutable_twist()->mutable_angular()->set_y(0.0);
    cmd_vel.mutable_twist()->mutable_angular()->set_z(result.cmd(1));
}
}  // namespace

double NMPCController::NormalizeAngle(double yaw) {
    while (yaw > M_PI) {
        yaw -= 2.0 * M_PI;
    }
    while (yaw < -M_PI) {
        yaw += 2.0 * M_PI;
    }
    return yaw;
}

void NMPCController::Configure(const proto::ControllerOptions& options,
                               std::string name,
                               std::shared_ptr<transform::Buffer> tf_buffer,
                               std::shared_ptr<map::costmap_2d::Costmap2DWrapper>
                                   costmap_wrapper) {
    plugin_name_ = std::move(name);
    tf_buffer_ = std::move(tf_buffer);
    costmap_wrapper_ = std::move(costmap_wrapper);
    base_options_ = ApplyDefaults(options.nmpc_controller_options());
    options_ = base_options_;
    initial_max_linear_vel_ = base_options_.max_linear_vel();

    path_transformer_.Configure(tf_buffer_, costmap_wrapper_, options_);
    collision_checker_.Configure(costmap_wrapper_, options_);
    optimizer_ = std::make_unique<NmpcOptimizer>(options_);

    AINFO << "Configured NMPC controller: " << plugin_name_
          << " solver=" << optimizer_->solver_type()
          << " horizon=" << options_.horizon_steps()
          << " dt=" << options_.model_dt()
          << " collision_check=" << (options_.enable_collision_check() ? "on"
                                                                        : "off");
}

void NMPCController::Cleanup() {
    optimizer_.reset();
    has_control_state_ = false;
}

void NMPCController::Activate() {}

void NMPCController::Deactivate() {
    AINFO << "Deactivating NMPC controller: " << plugin_name_;
}

void NMPCController::Reset() {
    path_progress_s_ = 0.0;
    has_control_state_ = false;
    last_goal_checker_ = nullptr;
    last_dist_to_goal_ = std::numeric_limits<double>::infinity();
    last_angle_to_goal_ = std::numeric_limits<double>::infinity();
    if (optimizer_) {
        optimizer_->Reset();
    }
}

void NMPCController::SetPlan(const automsgs::msgs::nav_msgs::Path& plan) {
    global_plan_ = plan;
    path_transformer_.SetPlan(global_plan_);
    path_progress_s_ = 0.0;
    has_control_state_ = false;
    last_dist_to_goal_ = std::numeric_limits<double>::infinity();
    last_angle_to_goal_ = std::numeric_limits<double>::infinity();
    if (optimizer_) {
        optimizer_->Reset();
    }
}

void NMPCController::SetSpeedLimit(const double& speed_limit,
                                     const bool& percentage) {
    speed_limit_ = speed_limit;
    speed_limit_percentage_ = percentage;
    options_ = base_options_;
    if (speed_limit == map::costmap_2d::NO_SPEED_LIMIT) {
        // Restore defaults from base options.
    } else if (percentage) {
        options_.set_max_linear_vel(
            std::max(initial_max_linear_vel_ * speed_limit / 100.0,
                     options_.min_linear_vel()));
    } else if (speed_limit > 0.0) {
        options_.set_max_linear_vel(
            std::max(speed_limit, options_.min_linear_vel()));
    }
    if (optimizer_) {
        optimizer_->UpdateOptions(options_);
    }
}

bool NMPCController::UpdateLocalPath(
    const automsgs::msgs::geometry_msgs::PoseStamped& pose) {
    const auto local_plan = path_transformer_.Transform(pose);
    if (local_plan.poses().empty()) {
        return false;
    }
    path_.SetPlan(local_plan);
    return true;
}

void NMPCController::CacheControlState(
    const automsgs::msgs::geometry_msgs::PoseStamped& pose,
    const automsgs::msgs::geometry_msgs::TwistStamped& velocity,
    common::GoalChecker* goal_checker) {
    last_robot_pose_ = pose;
    last_robot_velocity_ = velocity.twist();
    last_goal_pose_ = path_.GoalPose();
    last_goal_checker_ = goal_checker;
    has_control_state_ = true;

    const auto goal = path_.Goal();
    const double dx = pose.pose().position().x() - goal.x;
    const double dy = pose.pose().position().y() - goal.y;
    last_dist_to_goal_ = std::hypot(dx, dy);
    last_angle_to_goal_ =
        NormalizeAngle(YawFromPose(pose.pose()) - goal.yaw);
}

bool NMPCController::IsGoalReached(double dist_tolerance,
                                   double angle_tolerance) {
    if (has_control_state_ && last_goal_checker_ != nullptr) {
        return last_goal_checker_->IsGoalReached(
            last_robot_pose_.pose(), last_goal_pose_, last_robot_velocity_);
    }
    if (!std::isfinite(last_dist_to_goal_) ||
        !std::isfinite(last_angle_to_goal_)) {
        return false;
    }
    return last_dist_to_goal_ <= dist_tolerance &&
           std::fabs(last_angle_to_goal_) <= angle_tolerance;
}

double NMPCController::UpdatePathProgress(
    const automsgs::msgs::geometry_msgs::Pose& pose) {
    if (path_.empty()) {
        return 0.0;
    }

    const double search_behind = options_.path_search_window() > 0.0
                                     ? options_.path_search_window()
                                     : 1.0;
    const auto closest = path_.ClosestPointOnPath(
        pose.position().x(), pose.position().y(), path_progress_s_,
        search_behind);
    path_progress_s_ = std::max(path_progress_s_, closest.arc_length);

    const double lookahead =
        options_.lookahead_dist() > 0.0 ? options_.lookahead_dist() : 0.5;
    return std::min(path_.PathLength(), path_progress_s_ + lookahead);
}

bool NMPCController::SolveAndValidate(
    const DifferentialDriveProblem::StateVector& state, double ref_s,
    NmpcOptimizer::SolveResult* result) {
    if (!optimizer_->Solve(state, path_, ref_s, result)) {
        return false;
    }
    return collision_checker_.IsTrajectoryFree(result->predicted_states);
}

uint32 NMPCController::ComputeVelocityCommands(
    const automsgs::msgs::geometry_msgs::PoseStamped& pose,
    const automsgs::msgs::geometry_msgs::TwistStamped& velocity,
    automsgs::msgs::geometry_msgs::TwistStamped& cmd_vel,
    common::GoalChecker* goal_checker, std::string& message) {
    if (!optimizer_ || global_plan_.poses().empty()) {
        message = "NMPC: empty plan";
        return proto::CONTROLLER_RESULT_INVALID_PATH;
    }

    if (!UpdateLocalPath(pose)) {
        message = "NMPC: failed to transform path";
        return proto::CONTROLLER_RESULT_TF_ERROR;
    }

    CacheControlState(pose, velocity, goal_checker);
    if (goal_checker != nullptr &&
        goal_checker->IsGoalReached(pose.pose(), path_.GoalPose(),
                                    velocity.twist())) {
        SetZeroVelocity(cmd_vel);
        message.clear();
        return proto::CONTROLLER_RESULT_SUCCESS;
    }

    DifferentialDriveProblem::StateVector state;
    state(0) = pose.pose().position().x();
    state(1) = pose.pose().position().y();
    state(2) = YawFromPose(pose.pose());

    const double ref_s = UpdatePathProgress(pose.pose());

    // Scale velocity limits when approaching the terminal pose.
    proto::NMPCControllerOptions solve_options = options_;
    const double remaining = path_.PathLength() - path_progress_s_;
    const double approach_dist = options_.approach_velocity_scaling_dist();
    if (approach_dist > 0.0 && remaining < approach_dist) {
        const double scale = std::max(remaining / approach_dist, 0.0);
        const double min_vel =
            std::max(options_.min_approach_linear_vel(), options_.min_linear_vel());
        solve_options.set_max_linear_vel(
            std::max(min_vel, options_.max_linear_vel() * scale));
        optimizer_->UpdateOptions(solve_options);
    }

    NmpcOptimizer::SolveResult result;
    bool solved = SolveAndValidate(state, ref_s, &result);
    // Retry once at half speed if the rollout collides.
    if (!solved && options_.enable_collision_check()) {
        proto::NMPCControllerOptions reduced = solve_options;
        reduced.set_max_linear_vel(std::max(
            options_.min_linear_vel(), solve_options.max_linear_vel() * 0.5));
        reduced.set_max_angular_vel(solve_options.max_angular_vel() * 0.5);
        optimizer_->UpdateOptions(reduced);
        solved = SolveAndValidate(state, ref_s, &result);
        optimizer_->UpdateOptions(solve_options);
    }

    if (approach_dist > 0.0 && remaining < approach_dist) {
        optimizer_->UpdateOptions(options_);
    }

    if (!solved) {
        message = options_.enable_collision_check()
                      ? "NMPC: no collision-free command"
                      : "NMPC solve failed";
        return options_.enable_collision_check()
                   ? proto::CONTROLLER_RESULT_COLLISION
                   : proto::CONTROLLER_RESULT_NO_VALID_CMD;
    }

    ApplyCommand(result, cmd_vel);
    message.clear();
    return proto::CONTROLLER_RESULT_SUCCESS;
}

}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

// Plugins
CLASS_LOADER_REGISTER_CLASS(
    autonomy::control::controller::nmpc_controller::NMPCController,
    autonomy::control::common::ControllerInterface)

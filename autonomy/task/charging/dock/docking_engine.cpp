/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file docking_engine.cpp
 * @brief Tick-based port of opennav_docking::DockingServer (implementation).
 */

#include "autonomy/task/charging/dock/docking_engine.hpp"

#include <cmath>
#include <limits>

#include <automsgs/msgs/time_utils.hpp>
#include "autonomy/common/logging.hpp"
#include "autonomy/task/charging/dock/exceptions.hpp"
#include "autonomy/task/charging/dock/geometry.hpp"
#include "autonomy/task/charging/dock/utils.hpp"

namespace autonomy {
namespace task {
namespace charging {
namespace {

using Phase = ::autonomy::task::proto::DockPhase;
namespace status = ::automsgs::msgs::status_msgs;

::automsgs::msgs::status_msgs::StatusCode MapError(const std::exception& ex) {
    if (dynamic_cast<const DockNotInDB*>(&ex)) {
        return status::CHARGING_DOCK_NOT_FOUND;
    }
    if (dynamic_cast<const DockNotValid*>(&ex)) {
        return status::CHARGING_DOCK_NOT_VALID;
    }
    if (dynamic_cast<const FailedToStage*>(&ex)) {
        return status::CHARGING_FAILED_TO_STAGE;
    }
    if (dynamic_cast<const FailedToDetectDock*>(&ex)) {
        return status::CHARGING_FAILED_TO_DETECT;
    }
    if (dynamic_cast<const FailedToControl*>(&ex)) {
        return status::CHARGING_FAILED_TO_CONTROL;
    }
    if (dynamic_cast<const FailedToCharge*>(&ex)) {
        return status::CHARGING_FAILED_TO_CHARGE;
    }
    return status::CHARGING_DOCK_FAILED;
}

}  // namespace

bool DockingEngine::Configure(
    const std::shared_ptr<autolink::Node>& node,
    navigation::NavigationClient::Ptr navigation,
    const ::autonomy::task::proto::DockingOptions& options) {
    node_ = node;
    navigation_ = std::move(navigation);
    options_ = options;
    const double freq =
        options.controller_frequency() > 0.0 ? options.controller_frequency()
                                             : 50.0;
    cmd_period_ = std::chrono::milliseconds(
        static_cast<int>(std::max(1.0, 1000.0 / freq)));
    perception_timeout_ = options.initial_perception_timeout() > 0.0
                              ? options.initial_perception_timeout()
                              : 5.0;
    charge_timeout_ =
        options.wait_charge_timeout() > 0.0 ? options.wait_charge_timeout() : 5.0;
    approach_timeout_ = options.dock_approach_timeout() > 0.0
                            ? options.dock_approach_timeout()
                            : 30.0;
    undock_lin_ = options.undock_linear_tolerance() > 0.0
                      ? options.undock_linear_tolerance()
                      : 0.05;
    undock_ang_ = options.undock_angular_tolerance() > 0.0
                      ? options.undock_angular_tolerance()
                      : 0.05;
    max_retries_ = options.max_retries() > 0 ? options.max_retries() : 3;
    base_frame_ = options.base_frame().empty() ? "base_link" : options.base_frame();
    fixed_frame_ = options.fixed_frame().empty() ? "odom" : options.fixed_frame();
    dock_backwards_ = options.dock_backwards();
    prestaging_ = options.dock_prestaging_tolerance() > 0.0
                      ? options.dock_prestaging_tolerance()
                      : 0.5;
    controller_.Configure(node_, fixed_frame_, base_frame_, options.controller());
    navigator_.Configure(navigation_, {}, {});
    if (!db_.Initialize(node_, options_)) {
        return false;
    }
    if (node_) {
        const std::string topic = options.cmd_vel_topic().empty()
                                      ? "/cmd_vel"
                                      : options.cmd_vel_topic();
        cmd_writer_ = node_->CreateWriter<
            ::automsgs::msgs::geometry_msgs::TwistStamped>(topic);
    }
    return true;
}

void DockingEngine::Activate() { db_.Activate(); }

void DockingEngine::Deactivate() {
    Cancel();
    db_.Deactivate();
}

float DockingEngine::elapsed_sec() const {
    if (state_ == RunState::Idle) {
        return 0.f;
    }
    return static_cast<float>(
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_)
            .count());
}

bool DockingEngine::IsCharging() const {
    return dock_ && dock_->plugin && dock_->plugin->IsCharging();
}

float DockingEngine::BatteryPercent() const {
    if (dock_ && dock_->plugin) {
        return dock_->plugin->BatteryPercent();
    }
    return std::numeric_limits<float>::quiet_NaN();
}

std::string DockingEngine::SelectDockId(const std::string& preferred) const {
    if (!preferred.empty()) {
        const auto id = db_.SelectDockId(preferred);
        if (!id.empty()) {
            return id;
        }
    }
    ::automsgs::msgs::geometry_msgs::PoseStamped robot;
    if (geom::LookupRobotPose("map", base_frame_, &robot)) {
        const auto nearest = db_.NearestDockId(robot.pose().position().x(),
                                               robot.pose().position().y());
        if (!nearest.empty()) {
            return nearest;
        }
    }
    return db_.SelectDockId({});
}

bool DockingEngine::ReloadDatabase(const std::string& filepath) {
    return db_.ReloadFile(filepath);
}

void DockingEngine::PublishZero() {
    ::automsgs::msgs::geometry_msgs::Twist zero;
    PublishCmd(zero);
}

void DockingEngine::PublishCmd(const ::automsgs::msgs::geometry_msgs::Twist& twist) {
    if (!cmd_writer_) {
        return;
    }
    ::automsgs::msgs::geometry_msgs::TwistStamped msg;
    *msg.mutable_header()->mutable_stamp() =
        automsgs::msgs::builtin_interfaces::TimeNow();
    msg.mutable_header()->set_frame_id(base_frame_);
    *msg.mutable_twist() = twist;
    cmd_writer_->Write(msg);
}

void DockingEngine::SetFailed(const std::exception& ex) {
    error_code_ = MapError(ex);
    error_msg_ = ex.what();
    AERROR << "dock fail code=" << error_code_ << " " << error_msg_;
    PublishZero();
    navigator_.Cancel();
    state_ = RunState::Failed;
}

Dock* DockingEngine::ResolveDock(const DockRequest& request) {
    owns_temp_dock_ = false;
    if (request.use_dock_id && !request.dock_id.empty()) {
        return db_.FindDock(request.dock_id);
    }
    temp_dock_ = Dock{};
    temp_dock_.frame = request.dock_pose.header().frame_id().empty()
                           ? "map"
                           : request.dock_pose.header().frame_id();
    temp_dock_.pose = request.dock_pose.pose();
    temp_dock_.type = request.dock_type;
    temp_dock_.plugin = db_.FindDockPlugin(temp_dock_.type);
    if (!temp_dock_.plugin) {
        throw DockNotValid("No dock plugin for requested pose/type");
    }
    owns_temp_dock_ = true;
    return &temp_dock_;
}

bool DockingEngine::StartDock(const DockRequest& request) {
    std::lock_guard<std::mutex> lock(db_.mutex());
    Cancel();
    try {
        dock_ = ResolveDock(request);
        dock_id_ = request.dock_id;
        staging_pose_ = dock_->GetStagingPose();
        dock_pose_ = DockPoseStamped(*dock_);
        geom::TransformPose(dock_pose_, fixed_frame_, &dock_pose_);
        num_retries_ = 0;
        max_retries_ = request.max_retries > 0 ? request.max_retries : max_retries_;
        hold_until_percent_ = request.hold_until_percent;
        max_hold_sec_ = request.max_hold_sec;
        start_ = std::chrono::steady_clock::now();
        phase_start_ = start_;
        last_cmd_ = start_ - cmd_period_;
        state_ = RunState::Running;
        undocking_ = false;
        error_code_ = status::OK;
        error_msg_.clear();

        ::automsgs::msgs::geometry_msgs::PoseStamped robot;
        const bool have_robot =
            geom::LookupRobotPose(staging_pose_.header().frame_id(), base_frame_,
                                  &robot);
        const bool skip_nav =
            !request.navigate_to_staging ||
            (have_robot && geom::L2Norm(robot.pose(), staging_pose_.pose()) <
                               prestaging_);
        if (skip_nav) {
            AINFO << "dock skip staging nav (prestaging)";
            phase_ = Phase::DOCK_PHASE_INITIAL_PERCEPTION;
            phase_start_ = std::chrono::steady_clock::now();
        } else {
            phase_ = Phase::DOCK_PHASE_NAV_TO_STAGING;
            navigator_.Begin(
                staging_pose_,
                std::chrono::milliseconds(static_cast<int>(
                    std::max(1.0, request.max_staging_time_sec) * 1000.0)));
        }
        return true;
    } catch (const std::exception& ex) {
        SetFailed(ex);
        return false;
    }
}

bool DockingEngine::StartUndock(const UndockRequest& request) {
    std::lock_guard<std::mutex> lock(db_.mutex());
    Cancel();
    try {
        std::string type =
            request.dock_type.empty() ? curr_dock_type_ : request.dock_type;
        auto plugin = db_.FindDockPlugin(type);
        if (!plugin) {
            throw DockNotValid("No dock information to undock from");
        }
        temp_dock_ = Dock{};
        temp_dock_.plugin = plugin;
        temp_dock_.type = type;
        temp_dock_.frame = fixed_frame_;
        dock_ = &temp_dock_;
        owns_temp_dock_ = true;
        if (!geom::LookupRobotPose(fixed_frame_, base_frame_, &dock_pose_)) {
            throw FailedToControl("undock: robot pose unavailable");
        }
        temp_dock_.pose = dock_pose_.pose();
        staging_pose_ = plugin->GetStagingPose(dock_pose_.pose(), fixed_frame_);
        plugin->DisableCharging();
        undock_timeout_ = request.max_undocking_time_sec > 0.0
                              ? request.max_undocking_time_sec
                              : 30.0;
        start_ = std::chrono::steady_clock::now();
        phase_start_ = start_;
        last_cmd_ = start_ - cmd_period_;
        phase_ = Phase::DOCK_PHASE_UNDOCKING;
        undocking_ = true;
        state_ = RunState::Running;
        error_code_ = status::OK;
        AINFO << "undock type=" << type;
        return true;
    } catch (const std::exception& ex) {
        SetFailed(ex);
        return false;
    }
}

void DockingEngine::Cancel() {
    navigator_.Cancel();
    PublishZero();
    if (state_ == RunState::Running) {
        state_ = RunState::Canceled;
        error_code_ = status::CHARGING_CANCELLED;
    }
    dock_ = nullptr;
    owns_temp_dock_ = false;
}

bool DockingEngine::CommandToPose(
    const ::automsgs::msgs::geometry_msgs::PoseStamped& pose, double linear_tol,
    double angular_tol, bool is_docking, bool backward, bool* reached) {
    *reached = false;
    ::automsgs::msgs::geometry_msgs::PoseStamped robot;
    if (!geom::LookupRobotPose(pose.header().frame_id(), base_frame_, &robot)) {
        throw FailedToControl("robot pose unavailable");
    }
    const double dist = std::hypot(
        robot.pose().position().x() - pose.pose().position().x(),
        robot.pose().position().y() - pose.pose().position().y());
    const double yaw = geom::ShortestAngularDistance(geom::Yaw(robot.pose()),
                                                     geom::Yaw(pose.pose()));
    if (dist < linear_tol && std::fabs(yaw) < angular_tol) {
        *reached = true;
        return true;
    }
    ::automsgs::msgs::geometry_msgs::PoseStamped target;
    if (!geom::TransformPose(pose, base_frame_, &target)) {
        throw FailedToControl("failed to transform target into base frame");
    }
    ::automsgs::msgs::geometry_msgs::Twist cmd;
    if (!controller_.ComputeVelocity(target.pose(), &cmd, is_docking,
                                     backward)) {
        throw FailedToControl("Failed to get control");
    }
    PublishCmd(cmd);
    return true;
}

void DockingEngine::TickDock() {
    const auto now = std::chrono::steady_clock::now();
    switch (phase_) {
    case Phase::DOCK_PHASE_NAV_TO_STAGING: {
        const auto st = navigator_.Tick(false);
        if (st == DockNavigator::Status::Succeeded) {
            AINFO << "dock staged";
            phase_ = Phase::DOCK_PHASE_INITIAL_PERCEPTION;
            phase_start_ = now;
        }
        return;
    }
    case Phase::DOCK_PHASE_INITIAL_PERCEPTION: {
        if (!dock_->plugin->GetRefinedPose(&dock_pose_, dock_->id)) {
            if (std::chrono::duration<double>(now - phase_start_).count() >
                perception_timeout_) {
                throw FailedToDetectDock("Failed initial dock detection");
            }
            return;
        }
        AINFO << "dock initial detection ok";
        phase_ = Phase::DOCK_PHASE_CONTROLLING;
        phase_start_ = now;
        return;
    }
    case Phase::DOCK_PHASE_CONTROLLING: {
        if (dock_->plugin->IsDocked() || dock_->plugin->IsCharging()) {
            AINFO << "dock contact";
            phase_ = Phase::DOCK_PHASE_WAIT_FOR_CHARGE;
            phase_start_ = now;
            PublishZero();
            return;
        }
        if (!dock_->plugin->GetRefinedPose(&dock_pose_, dock_->id)) {
            throw FailedToDetectDock("Failed dock detection");
        }
        auto target = dock_pose_;
        if (dock_backwards_) {
            geom::SetYaw(target.mutable_pose(),
                         geom::NormalizeAngle(geom::Yaw(target.pose()) + M_PI));
        }
        const double yaw = geom::Yaw(target.pose());
        constexpr double kBackwardProjection = 0.25;
        target.mutable_pose()->mutable_position()->set_x(
            target.pose().position().x() + std::cos(yaw) * kBackwardProjection);
        target.mutable_pose()->mutable_position()->set_y(
            target.pose().position().y() + std::sin(yaw) * kBackwardProjection);
        ::automsgs::msgs::geometry_msgs::PoseStamped in_base;
        if (!geom::TransformPose(target, base_frame_, &in_base)) {
            throw FailedToControl("Failed to transform dock pose to base");
        }
        ::automsgs::msgs::geometry_msgs::Twist cmd;
        if (!controller_.ComputeVelocity(in_base.pose(), &cmd, true,
                                         dock_backwards_)) {
            throw FailedToControl("Failed to get control");
        }
        PublishCmd(cmd);
        if (std::chrono::duration<double>(now - phase_start_).count() >
            approach_timeout_) {
            throw FailedToControl(
                "Timed out approaching dock; dock nor charging detected");
        }
        return;
    }
    case Phase::DOCK_PHASE_WAIT_FOR_CHARGE: {
        if (dock_->plugin->IsCharging()) {
            curr_dock_type_ = dock_->type;
            PublishZero();
            if (hold_until_percent_ > 0.f) {
                AINFO << "dock charging, holding until soc="
                      << hold_until_percent_ << "%";
                phase_ = Phase::DOCK_PHASE_HOLD_CHARGE;
                phase_start_ = now;
                return;
            }
            state_ = RunState::Succeeded;
            AINFO << "dock charging";
            return;
        }
        if (std::chrono::duration<double>(now - phase_start_).count() >
            charge_timeout_) {
            throw FailedToCharge("Timed out waiting for charge to start");
        }
        return;
    }
    case Phase::DOCK_PHASE_HOLD_CHARGE: {
        PublishZero();
        if (!dock_->plugin->IsCharging()) {
            throw FailedToCharge("lost charge while holding on dock");
        }
        const float soc = dock_->plugin->BatteryPercent();
        if (std::isfinite(soc) && soc >= hold_until_percent_) {
            state_ = RunState::Succeeded;
            AINFO << "dock charge complete soc=" << soc << "%";
            return;
        }
        if (max_hold_sec_ > 0.0 &&
            std::chrono::duration<double>(now - phase_start_).count() >
                max_hold_sec_) {
            AWARN << "dock hold-charge timeout; treating as success soc="
                  << soc;
            state_ = RunState::Succeeded;
        }
        return;
    }
    case Phase::DOCK_PHASE_RETRY: {
        bool reached = false;
        CommandToPose(staging_pose_, undock_lin_, undock_ang_, false,
                      !dock_backwards_, &reached);
        if (reached) {
            AINFO << "dock retry from staging retries=" << num_retries_;
            phase_ = Phase::DOCK_PHASE_INITIAL_PERCEPTION;
            phase_start_ = now;
        } else if (std::chrono::duration<double>(now - phase_start_).count() >
                   approach_timeout_) {
            throw FailedToControl("Timed out resetting dock approach");
        }
        return;
    }
    default:
        return;
    }
}

void DockingEngine::TickUndock() {
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - start_).count() > undock_timeout_) {
        throw FailedToControl("Undocking timed out");
    }
    if (dock_->plugin && !dock_->plugin->DisableCharging()) {
        return;
    }
    bool reached = false;
    CommandToPose(staging_pose_, undock_lin_, undock_ang_, false,
                  !dock_backwards_, &reached);
    if (!reached) {
        return;
    }
    if (dock_->plugin && !dock_->plugin->HasStoppedCharging()) {
        throw FailedToControl(
            "Failed to control off dock, still charging");
    }
    PublishZero();
    curr_dock_type_.clear();
    state_ = RunState::Succeeded;
    AINFO << "undock complete";
}

DockingEngine::RunState DockingEngine::Tick() {
    if (state_ != RunState::Running) {
        return state_;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now - last_cmd_ < cmd_period_) {
        return state_;
    }
    last_cmd_ = now;
    const bool retryable =
        !undocking_ && (phase_ == Phase::DOCK_PHASE_CONTROLLING ||
                        phase_ == Phase::DOCK_PHASE_WAIT_FOR_CHARGE ||
                        phase_ == Phase::DOCK_PHASE_HOLD_CHARGE);
    try {
        std::lock_guard<std::mutex> lock(db_.mutex());
        if (undocking_) {
            TickUndock();
        } else {
            TickDock();
        }
    } catch (const DockingException& ex) {
        if (retryable && ++num_retries_ <= max_retries_) {
            AWARN << "Docking failed, will retry: " << ex.what();
            phase_ = Phase::DOCK_PHASE_RETRY;
            phase_start_ = now;
            navigator_.Cancel();
            return state_;
        }
        SetFailed(ex);
    } catch (const std::exception& ex) {
        SetFailed(ex);
    }
    return state_;
}

}  // namespace charging
}  // namespace task
}  // namespace autonomy

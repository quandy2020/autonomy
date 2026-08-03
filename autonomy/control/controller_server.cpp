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

#include "autonomy/control/controller_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>

#include "autolink/node/writer.hpp"
#include "autonomy/common/logging.hpp"

#include "autonomy/control/checker/simple_goal_checker.hpp"
#include "autonomy/control/checker/simple_progress_checker.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/control/constants.hpp"
#include "autonomy/control/controller/graceful_controller/controller.hpp"
#include "autonomy/control/controller/mppi_controller/controller.hpp"
#include "autonomy/control/controller/pure_pursuit_controller/controller.hpp"
#include "autonomy/control/proto/controller_options.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace control {
namespace {

using Time = automsgs::msgs::builtin_interfaces::Time;

struct PluginSpec {
    std::string id;
    std::string type;
};

const std::unordered_map<std::string, std::string>& ControllerClassAliases() {
    static const std::unordered_map<std::string, std::string> kAliases = {
        {"mppi_controller", "MppiController"},
        {"graceful_controller", "GracefulController"},
        {"regulated_pure_pursuit", "RegulatedPurePursuitController"},
        {"pure_pursuit", "RegulatedPurePursuitController"},
        {"rpp", "RegulatedPurePursuitController"},
    };
    return kAliases;
}

std::string ResolveControllerClass(const std::string& plugin_id) {
    const auto& aliases = ControllerClassAliases();
    const auto it = aliases.find(plugin_id);
    return it != aliases.end() ? it->second : plugin_id;
}

std::vector<PluginSpec> ParsePluginSpecs(
    const google::protobuf::RepeatedPtrField<std::string>& entries) {
    std::vector<PluginSpec> specs;
    specs.reserve(static_cast<size_t>(entries.size()));
    for (const auto& entry_str : entries) {
        if (entry_str.empty()) {
            continue;
        }
        PluginSpec spec;
        const size_t colon = entry_str.find(':');
        if (colon != std::string::npos) {
            spec.id = entry_str.substr(0, colon);
            spec.type = entry_str.substr(colon + 1);
        } else {
            spec.id = entry_str;
            spec.type = ResolveControllerClass(entry_str);
        }
        if (!spec.id.empty() && !spec.type.empty()) {
            specs.push_back(std::move(spec));
        }
    }
    return specs;
}

Time NowTime() {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(now);
    const auto nsec =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now - sec);
    Time stamp;
    stamp.set_sec(static_cast<int32_t>(sec.count()));
    stamp.set_nanosec(static_cast<uint32_t>(nsec.count()));
    return stamp;
}

double TimeDiffSeconds(const Time& a, const Time& b) {
    return static_cast<double>(a.sec() - b.sec()) +
           static_cast<double>(a.nanosec() - b.nanosec()) * 1e-9;
}

}  // namespace

ControllerServer::ControllerServer(const proto::ControllerOptions& options)
    : options_{options} {
    if (options_.has_costmap_2d_options() &&
        options_.costmap_2d_options().enabled()) {
        costmap_wrapper_ = std::make_shared<map::costmap_2d::Costmap2DWrapper>(
            options_.costmap_2d_options(), "local_costmap");
    }

    controller_frequency_ = options_.controller_frequency() > 0.0
                                ? options_.controller_frequency()
                                : 20.0;
    failure_tolerance_ = options_.failure_tolerance() > 0.0
                             ? options_.failure_tolerance()
                             : 0.0;
    publish_zero_velocity_ = options_.publish_zero_velocity();

    if (options_.has_checker_options() &&
        options_.checker_options().has_goal_checker()) {
        goal_reached_tolerance_ =
            options_.checker_options().goal_checker().xy_goal_tolerance() > 0.0
                ? options_.checker_options().goal_checker().xy_goal_tolerance()
                : 0.25;
    }

    costmap_update_timeout_ =
        automsgs::msgs::builtin_interfaces::DurationFromSeconds(300.0);

    tf_buffer_ = std::shared_ptr<transform::Buffer>(
        transform::Buffer::Instance(), [](transform::Buffer*) {});
    odom_smoother_ = std::make_shared<utils::OdomSmoother>();
}

ControllerServer::~ControllerServer() {
    Shutdown();
}

void ControllerServer::LoadPlugins() {
    if (plugins_loaded_) {
        return;
    }

    controllers_.clear();
    controller_ids_.clear();
    controller_ids_concat_.clear();
    goal_checkers_.clear();
    default_goal_checker_ids_.clear();
    progress_checkers_.clear();
    default_progress_checker_ids_.clear();

    std::vector<PluginSpec> specs;
    if (options_.controller_plugins_size() > 0) {
        specs = ParsePluginSpecs(options_.controller_plugins());
    } else {
        specs = {{"mppi_controller", "MppiController"}};
    }

    for (const auto& spec : specs) {
        if (controllers_.count(spec.id) > 0) {
            AWARN << "Duplicate controller plugin id ignored: " << spec.id;
            continue;
        }

        const std::string resolved = ResolveControllerClass(spec.type);
        common::ControllerInterface::SharedPtr instance;

        if (resolved == "MppiController" || resolved == "MPPIController") {
            auto ctrl =
                std::make_shared<controller::mppi_controller::MPPIController>();
            ctrl->Configure(options_, spec.id, tf_buffer_, costmap_wrapper_);
            ctrl->Activate();
            instance = std::move(ctrl);
        } else if (resolved == "GracefulController") {
            auto ctrl = std::make_shared<controller::GracefulController>();
            ctrl->Configure(options_, spec.id, tf_buffer_, costmap_wrapper_);
            ctrl->Activate();
            instance = std::move(ctrl);
        } else if (resolved == "RegulatedPurePursuitController") {
            auto ctrl = std::make_shared<
                controller::pure_pursuit_controller::RegulatedPurePursuitController>();
            ctrl->Configure(options_, spec.id, tf_buffer_, costmap_wrapper_);
            ctrl->Activate();
            instance = std::move(ctrl);
        } else {
            AWARN << "Unknown controller plugin type: " << resolved
                  << " for id: " << spec.id;
            continue;
        }

        controllers_.insert({spec.id, instance});
        controller_ids_.push_back(spec.id);
        if (!controller_ids_concat_.empty()) {
            controller_ids_concat_ += ", ";
        }
        controller_ids_concat_ += spec.id;
        AINFO << "Created controller plugin: " << spec.id
              << " (type = " << resolved << ")";
    }

    if (controllers_.empty()) {
        AFATAL << "No controller plugins loaded";
        return;
    }

    default_ids_ = controller_ids_;

    const auto& checker_opts = options_.checker_options();
    auto goal_checker = std::make_shared<checker::SimpleGoalChecker>();
    goal_checker->Initialize("goal_checker", costmap_wrapper_);
    if (checker_opts.has_goal_checker()) {
        const auto& gc = checker_opts.goal_checker();
        goal_checker->SetTolerances(
            gc.xy_goal_tolerance() > 0.0 ? gc.xy_goal_tolerance() : 0.25,
            gc.yaw_goal_tolerance() > 0.0 ? gc.yaw_goal_tolerance() : 0.35,
            gc.stateful(),
            gc.path_length_tolerance() > 0.0 ? gc.path_length_tolerance()
                                             : 1.0);
    } else {
        goal_checker->SetTolerances(goal_reached_tolerance_, 0.35, true);
    }
    goal_checkers_.insert({"goal_checker", goal_checker});
    default_goal_checker_ids_.push_back("goal_checker");

    auto progress_checker = std::make_shared<checker::SimpleProgressChecker>();
    progress_checker->Initialize("progress_checker");
    if (checker_opts.has_progress_checker()) {
        const auto& pc = checker_opts.progress_checker();
        progress_checker->SetRequiredMovementRadius(
            pc.required_movement_radius() > 0.0
                ? pc.required_movement_radius()
                : 0.5);
    }
    progress_checkers_.insert({"progress_checker", progress_checker});
    default_progress_checker_ids_.push_back("progress_checker");

    if (costmap_wrapper_) {
        global_frame_ = costmap_wrapper_->getGlobalFrameID();
        robot_base_frame_ = costmap_wrapper_->getBaseFrameID();
    }

    plugins_loaded_ = true;
    AINFO << "ControllerServer plugins loaded: controllers=["
          << controller_ids_concat_ << "]";
}

void ControllerServer::Start() {
    if (costmap_wrapper_) {
        costmap_wrapper_->Start();
    }

    if (!node_) {
        node_ = autolink::CreateNode(kControllerServerNodeName);
    }
    if (node_) {
        AttachAutolinkNode(node_);
    }

    if (costmap_wrapper_) {
        LoadPlugins();
    }
}

void ControllerServer::Shutdown() {
    DetachAutolinkNode();
    cmd_vel_writer_.reset();
    node_.reset();
    if (costmap_wrapper_) {
        costmap_wrapper_->Stop();
    }
    OnGoalExit();
}

void ControllerServer::SetSharedCostmap(
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap) {
    if (!costmap) {
        return;
    }
    costmap_wrapper_ = std::move(costmap);
    global_frame_ = costmap_wrapper_->getGlobalFrameID();
    robot_base_frame_ = costmap_wrapper_->getBaseFrameID();
    plugins_loaded_ = false;
    LoadPlugins();
}

void ControllerServer::UpdateOdometry(
    const automsgs::msgs::planning_msgs::Odometry& odom) {
    if (odom_smoother_) {
        odom_smoother_->UpdateOdometry(odom);
    }
}

bool ControllerServer::GetLatestOdometry(
    automsgs::msgs::planning_msgs::Odometry& odom) const {
    return odom_smoother_ && odom_smoother_->GetLatestOdometry(odom);
}

bool ControllerServer::FindControllerId(const std::string& c_name,
                                        std::string& current_controller) {
    auto resolve_alias = [&](const std::string& name) -> std::string {
        if (name == "FollowPath" || name == "follow_path") {
            return controller_ids_.empty() ? std::string() : controller_ids_.front();
        }
        return name;
    };

    const std::string requested = resolve_alias(c_name);
    if (!requested.empty()) {
        if (controllers_.count(requested) == 0) {
            AWARN << "Controller id '" << requested << "' not found";
            return false;
        }
        current_controller = requested;
        return true;
    }
    if (!default_ids_.empty()) {
        current_controller = default_ids_.front();
        return true;
    }
    return false;
}

bool ControllerServer::FindGoalCheckerId(
    const std::string& c_name, std::string& current_goal_checker) {
    auto resolve_alias = [&](const std::string& name) -> std::string {
        if (name.empty() || name == "goal_checker") {
            return default_goal_checker_ids_.empty()
                       ? std::string()
                       : default_goal_checker_ids_.front();
        }
        return name;
    };

    const std::string requested = resolve_alias(c_name);
    if (!requested.empty()) {
        if (goal_checkers_.count(requested) == 0) {
            AWARN << "Goal checker id '" << requested << "' not found";
            return false;
        }
        current_goal_checker = requested;
        return true;
    }
    return false;
}

bool ControllerServer::FindProgressCheckerId(
    const std::string& c_name, std::string& current_progress_checker) {
    auto resolve_alias = [&](const std::string& name) -> std::string {
        if (name.empty() || name == "progress_checker") {
            return default_progress_checker_ids_.empty()
                       ? std::string()
                       : default_progress_checker_ids_.front();
        }
        return name;
    };

    const std::string requested = resolve_alias(c_name);
    if (!requested.empty()) {
        if (progress_checkers_.count(requested) == 0) {
            AWARN << "Progress checker id '" << requested << "' not found";
            return false;
        }
        current_progress_checker = requested;
        return true;
    }
    return false;
}

common::ControllerInterface* ControllerServer::GetActiveController() {
    if (current_controller_.empty() ||
        controllers_.count(current_controller_) == 0) {
        return nullptr;
    }
    return controllers_[current_controller_].get();
}

common::GoalChecker* ControllerServer::GetActiveGoalChecker() {
    if (current_goal_checker_.empty() ||
        goal_checkers_.count(current_goal_checker_) == 0) {
        return nullptr;
    }
    return goal_checkers_[current_goal_checker_].get();
}

common::ProgressChecker* ControllerServer::GetActiveProgressChecker() {
    if (current_progress_checker_.empty() ||
        progress_checkers_.count(current_progress_checker_) == 0) {
        return nullptr;
    }
    return progress_checkers_[current_progress_checker_].get();
}

void ControllerServer::PublishVelocity(
    const automsgs::msgs::geometry_msgs::TwistStamped& cmd_vel) {
    last_cmd_vel_ = cmd_vel;
    if (cmd_vel_writer_) {
        cmd_vel_writer_->Write(cmd_vel);
    }
}

void ControllerServer::PublishZeroVelocity() {
    automsgs::msgs::geometry_msgs::TwistStamped zero;
    *zero.mutable_header()->mutable_stamp() = NowTime();
    zero.mutable_header()->set_frame_id( robot_base_frame_);
    PublishVelocity(zero);
}

void ControllerServer::ComputeControl() {
    if (!plugins_loaded_) {
        LoadPlugins();
    }

    if (current_path_.poses().empty()) {
        throw common::InvalidPath("FollowPath received an empty path");
    }

    if (!FindControllerId(follow_controller_id_, current_controller_)) {
        throw common::InvalidController("No valid controller for FollowPath");
    }
    if (!FindGoalCheckerId(follow_goal_checker_id_, current_goal_checker_)) {
        throw common::InvalidController("No valid goal checker for FollowPath");
    }
    if (!FindProgressCheckerId(follow_progress_checker_id_,
                               current_progress_checker_)) {
        throw common::InvalidController(
            "No valid progress checker for FollowPath");
    }

    auto* controller = GetActiveController();
    auto* goal_checker = GetActiveGoalChecker();
    auto* progress_checker = GetActiveProgressChecker();
    if (!controller || !goal_checker || !progress_checker) {
        throw common::InvalidController("ControllerServer plugins not ready");
    }

    goal_checker->Reset();
    progress_checker->Reset();
    controller->Reset();
    controller->SetPlan(current_path_);

    if (!current_path_.poses().empty()) {
        end_pose_ = current_path_.poses(current_path_.poses_size() - 1);
    }

    last_valid_cmd_time_ = NowTime();
    follow_path_active_ = true;
    controllers_active_ = true;

    AINFO << "FollowPath started: controller=" << current_controller_
          << " path_poses=" << current_path_.poses_size();
}

void ControllerServer::ComputeAndPublishVelocity() {
    if (!follow_path_active_) {
        return;
    }

    auto* controller = GetActiveController();
    auto* goal_checker = GetActiveGoalChecker();
    auto* progress_checker = GetActiveProgressChecker();
    if (!controller || !goal_checker || !progress_checker) {
        throw common::InvalidController("Active controller/checker missing");
    }

    automsgs::msgs::geometry_msgs::PoseStamped pose;
    if (!GetRobotPose(pose)) {
        throw common::ControllerTFError("Failed to obtain robot pose");
    }

    automsgs::msgs::planning_msgs::Odometry odom;
    automsgs::msgs::geometry_msgs::TwistStamped velocity;
    velocity.mutable_header()->set_frame_id( robot_base_frame_);
    if (GetLatestOdometry(odom)) {
        *velocity.mutable_header()->mutable_stamp() = odom.header().stamp();
        *velocity.mutable_twist() = odom.twist().twist();
    } else {
        *velocity.mutable_header()->mutable_stamp() = pose.header().stamp();
    }

    if (!progress_checker->Check(pose)) {
        throw common::FailedToMakeProgress(
            "Failed to make progress towards goal");
    }

    automsgs::msgs::geometry_msgs::TwistStamped cmd_vel;
    std::string message;

    std::unique_lock<map::costmap_2d::Costmap2D::mutex_t> costmap_lock;
    if (costmap_wrapper_ && costmap_wrapper_->getCostmap()) {
        costmap_lock = std::unique_lock<map::costmap_2d::Costmap2D::mutex_t>(
            *(costmap_wrapper_->getCostmap()->getMutex()));
    }

    const uint32_t code = controller->ComputeVelocityCommands(
        pose, velocity, cmd_vel, goal_checker, message);

    if (code != proto::CONTROLLER_RESULT_SUCCESS) {
        if (failure_tolerance_ > 0.0) {
            const double elapsed =
                TimeDiffSeconds(NowTime(), last_valid_cmd_time_);
            if (elapsed > failure_tolerance_) {
                if (publish_zero_velocity_) {
                    PublishZeroVelocity();
                }
                throw common::PatienceExceeded(
                    "Controller patience exceeded: " + message);
            }
            AWARN << "Controller returned code=" << code
                  << " (within failure_tolerance): " << message;
            return;
        }
        if (publish_zero_velocity_) {
            PublishZeroVelocity();
        }
        throw common::ControllerException("Controller failed: " + message);
    }

    last_valid_cmd_time_ = NowTime();
    if (cmd_vel.header().frame_id().empty()) {
        cmd_vel.mutable_header()->set_frame_id( robot_base_frame_);
    }
    if (cmd_vel.header().stamp().sec() == 0 && cmd_vel.header().stamp().nanosec() == 0) {
        *cmd_vel.mutable_header()->mutable_stamp() = NowTime();
    }
    PublishVelocity(cmd_vel);
}

void ControllerServer::UpdateGlobalPath() {
    if (!follow_path_active_ || current_path_.poses().empty()) {
        return;
    }
    auto* controller = GetActiveController();
    if (controller) {
        controller->SetPlan(current_path_);
    }
}

void ControllerServer::OnGoalExit() {
    follow_path_active_ = false;
    controllers_active_ = false;
    follow_path_is_closed_ = false;
    follow_has_last_travel_pose_ = false;
    follow_path_length_ = 0.0;
    follow_distance_traveled_ = 0.0;
    follow_min_distance_before_goal_ = 0.0;
    follow_controller_id_.clear();
    follow_goal_checker_id_.clear();
    follow_progress_checker_id_.clear();
    current_controller_.clear();
    current_goal_checker_.clear();
    current_progress_checker_.clear();

    if (publish_zero_velocity_) {
        PublishZeroVelocity();
    }
}

bool ControllerServer::IsGoalReached() {
    if (!follow_path_active_ || current_path_.poses().empty()) {
        return false;
    }

    auto* goal_checker = GetActiveGoalChecker();
    if (!goal_checker) {
        return false;
    }

    automsgs::msgs::geometry_msgs::PoseStamped pose;
    if (!GetRobotPose(pose)) {
        return false;
    }

    automsgs::msgs::planning_msgs::Odometry odom;
    automsgs::msgs::geometry_msgs::Twist velocity;
    if (GetLatestOdometry(odom)) {
        velocity = odom.twist().twist();
    }

    const auto& goal_pose = current_path_.poses(current_path_.poses_size() - 1).pose();
    return goal_checker->IsGoalReached(pose.pose(), goal_pose, velocity);
}

bool ControllerServer::GetRobotPose(
    automsgs::msgs::geometry_msgs::PoseStamped& pose) {
    if (costmap_wrapper_ && costmap_wrapper_->getRobotPose(pose)) {
        return true;
    }

    if (odom_smoother_) {
        automsgs::msgs::planning_msgs::Odometry odom;
        if (odom_smoother_->GetLatestOdometry(odom) &&
            !odom.header().frame_id().empty()) {
            pose.mutable_header()->set_frame_id(
                odom.header().frame_id().empty() ? global_frame_
                                             : odom.header().frame_id());
            *pose.mutable_header()->mutable_stamp() = odom.header().stamp();
            *pose.mutable_pose() = odom.pose().pose().pose();
            return true;
        }
    }
    return false;
}

}  // namespace control
}  // namespace autonomy

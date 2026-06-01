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

#include "autonomy/planning/planner_server.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "autolink/node/node.hpp"
#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/common/time.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/footprint_collision_checker.hpp"
#include "autonomy/planning/common/planner_exceptions.hpp"
#include "autonomy/planning/constants.hpp"
#include "autonomy/planning/utils/geometry_utils.hpp"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace planning {

using Time = commsgs::builtin_interfaces::Time;

namespace {

using PluginPm = autolink::plugin_manager::PluginManager;

struct PluginSpec {
    std::string id;
    std::string type;
};

const std::unordered_map<std::string, std::string>& PlannerClassAliases() {
    static const std::unordered_map<std::string, std::string> kAliases = {
        {"navfn_planner", "NavfnPlanner"},
        {"dijkstra_planner", "DijkstraPlanner"},
        {"theta_star_planner", "ThetaStarPlanner"},
    };
    return kAliases;
}

std::string ResolvePlannerClass(const std::string& plugin_id) {
    const auto& aliases = PlannerClassAliases();
    const auto it = aliases.find(plugin_id);
    return it != aliases.end() ? it->second : plugin_id;
}

std::vector<PluginSpec> ParsePluginSpecs(
    const std::vector<std::string>& entries) {
    std::vector<PluginSpec> specs;
    specs.reserve(entries.size());
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
            spec.type = ResolvePlannerClass(entry_str);
        }
        if (!spec.id.empty() && !spec.type.empty()) {
            specs.push_back(std::move(spec));
        }
    }
    return specs;
}

void LoadExternalPlugins(const proto::PlannerOptions& options) {
    static bool loaded = false;
    if (loaded) {
        return;
    }
    auto* pm = PluginPm::Instance();
    pm->RegisterInProcessClass<common::GlobalPlanner>("NavfnPlanner");
    pm->RegisterInProcessClass<common::GlobalPlanner>("DijkstraPlanner");
    pm->RegisterInProcessClass<common::GlobalPlanner>("ThetaStarPlanner");
    for (const auto& path : options.planner_plugin_libraries()) {
        if (path.empty()) {
            continue;
        }
        if (!pm->LoadPlugin(path)) {
            AWARN << "Failed to load plugin description: " << path;
        }
    }
    pm->LoadInstalledPlugins();
    loaded = true;
}

bool IsPlannerClassRegistered(const std::string& type) {
    const std::string resolved = ResolvePlannerClass(type);
    const auto names =
        PluginPm::Instance()->GetDerivedClassNameByBaseClass<
            common::GlobalPlanner>();
    return std::find(names.begin(), names.end(), resolved) != names.end();
}

common::GlobalPlanner::SharedPtr CreatePlannerInstance(
    const std::string& type) {
    auto instance = PluginPm::Instance()->CreateInstance<common::GlobalPlanner>(
        ResolvePlannerClass(type));
    return instance ? common::GlobalPlanner::SharedPtr(std::move(instance))
                    : nullptr;
}

task_proto::ComputePathToPoseErrorCode MapToComputePathToPoseErrorCode(
    const std::exception& ex) {
    if (dynamic_cast<const common::InvalidPlanner*>(&ex)) {
        return task_proto::COMPUTE_PATH_TO_POSE_INVALID_PLANNER;
    }
    if (dynamic_cast<const common::StartOccupied*>(&ex)) {
        return task_proto::COMPUTE_PATH_TO_POSE_START_OCCUPIED;
    }
    if (dynamic_cast<const common::GoalOccupied*>(&ex)) {
        return task_proto::COMPUTE_PATH_TO_POSE_GOAL_OCCUPIED;
    }
    if (dynamic_cast<const common::NoValidPathCouldBeFound*>(&ex)) {
        return task_proto::COMPUTE_PATH_TO_POSE_NO_VALID_PATH;
    }
    if (dynamic_cast<const common::PlannerTimedOut*>(&ex)) {
        return task_proto::COMPUTE_PATH_TO_POSE_TIMEOUT;
    }
    if (dynamic_cast<const common::StartOutsideMapBounds*>(&ex)) {
        return task_proto::COMPUTE_PATH_TO_POSE_START_OUTSIDE_MAP;
    }
    if (dynamic_cast<const common::GoalOutsideMapBounds*>(&ex)) {
        return task_proto::COMPUTE_PATH_TO_POSE_GOAL_OUTSIDE_MAP;
    }
    if (dynamic_cast<const common::PlannerTFError*>(&ex)) {
        return task_proto::COMPUTE_PATH_TO_POSE_TF_ERROR;
    }
    return task_proto::COMPUTE_PATH_TO_POSE_UNKNOWN;
}

task_proto::ComputePathThroughPosesErrorCode
MapToComputePathThroughPosesErrorCode(const std::exception& ex) {
    if (dynamic_cast<const common::InvalidPlanner*>(&ex)) {
        return task_proto::COMPUTE_PATH_THROUGH_POSES_INVALID_PLANNER;
    }
    if (dynamic_cast<const common::StartOccupied*>(&ex)) {
        return task_proto::COMPUTE_PATH_THROUGH_POSES_START_OCCUPIED;
    }
    if (dynamic_cast<const common::GoalOccupied*>(&ex)) {
        return task_proto::COMPUTE_PATH_THROUGH_POSES_GOAL_OCCUPIED;
    }
    if (dynamic_cast<const common::NoValidPathCouldBeFound*>(&ex)) {
        return task_proto::COMPUTE_PATH_THROUGH_POSES_NO_VALID_PATH;
    }
    if (dynamic_cast<const common::PlannerTimedOut*>(&ex)) {
        return task_proto::COMPUTE_PATH_THROUGH_POSES_TIMEOUT;
    }
    if (dynamic_cast<const common::StartOutsideMapBounds*>(&ex)) {
        return task_proto::COMPUTE_PATH_THROUGH_POSES_START_OUTSIDE_MAP;
    }
    if (dynamic_cast<const common::GoalOutsideMapBounds*>(&ex)) {
        return task_proto::COMPUTE_PATH_THROUGH_POSES_GOAL_OUTSIDE_MAP;
    }
    if (dynamic_cast<const common::PlannerTFError*>(&ex)) {
        return task_proto::COMPUTE_PATH_THROUGH_POSES_TF_ERROR;
    }
    if (dynamic_cast<const common::NoViapointsGiven*>(&ex)) {
        return task_proto::COMPUTE_PATH_THROUGH_POSES_NO_WAYPOINTS;
    }
    return task_proto::COMPUTE_PATH_THROUGH_POSES_UNKNOWN;
}

void NormalizePoseFrame(commsgs::geometry_msgs::PoseStamped& pose,
                          const std::string& global_frame) {
    if (pose.header.frame_id.empty()) {
        AWARN << "Pose has empty frame_id; assuming costmap global frame '"
              << global_frame << "'";
        pose.header.frame_id = global_frame;
    }
}

void ThrowOnPlannerResultCode(uint32_t return_code,
                              const std::string& planner_id) {
    using proto::PlannerResultCode;
    switch (return_code) {
        case PlannerResultCode::PLANNER_SUCCESS:
            return;
        case PlannerResultCode::PLANNER_CANCELED:
            throw common::PlannerCancelled("Planner " + planner_id +
                                           " was cancelled");
        case PlannerResultCode::PLANNER_BLOCKED_START:
            throw common::StartOccupied("Start is occupied");
        case PlannerResultCode::PLANNER_BLOCKED_GOAL:
            throw common::GoalOccupied("Goal is occupied");
        case PlannerResultCode::PLANNER_NO_PATH_FOUND:
        case PlannerResultCode::PLANNER_EMPTY_PATH:
            throw common::NoValidPathCouldBeFound("Planner " + planner_id +
                                                  " failed to find a path");
        case PlannerResultCode::PLANNER_PAT_EXCEEDED:
            throw common::PlannerTimedOut("Planner " + planner_id +
                                          " exceeded allowed planning time");
        case PlannerResultCode::PLANNER_TF_ERROR:
            throw common::PlannerTFError("Planner " + planner_id +
                                         " reported a TF error");
        case PlannerResultCode::PLANNER_INVALID_PLUGIN:
        case PlannerResultCode::PLANNER_NOT_INITIALIZED:
            throw common::InvalidPlanner("Planner id " + planner_id +
                                         " is invalid or not initialized");
        default:
            throw common::PlannerException("Planner " + planner_id +
                                           " failed with code " +
                                           std::to_string(return_code));
    }
}

}  // namespace

PlannerServer::PlannerServer(const proto::PlannerOptions& options)
    : options_{options} {
    costmap_wrapper_ = std::make_shared<map::costmap_2d::Costmap2DWrapper>(
        options_.costmap(), kCostmapTopicName);
    if (!costmap_wrapper_) {
        AFATAL << "Failed to configure costmap wrapper.";
        return;
    }

    costmap_ = costmap_wrapper_->getCostmap();

    default_planner_id_ = options_.default_planner_id().empty()
                              ? "navfn_planner"
                              : options_.default_planner_id();

    LoadPlannerPlugins();

    const double expected_planner_frequency = options_.expected_planner_frequency();
    if (expected_planner_frequency > 0.0) {
        max_planner_duration_ = 1.0 / expected_planner_frequency;
    } else {
        AWARN << "expected_planner_frequency is " << expected_planner_frequency
              << " Hz; must be > 0 to enable duration overrun warnings.";
        max_planner_duration_ = 0.0;
    }

    costmap_wrapper_->Start();

    for (auto& entry : planners_) {
        entry.second->Activate();
    }

    node_ = autolink::CreateNode(kPlannerServerNodeName);
    if (!node_) {
        AWARN << "PlannerServer: autolink node creation failed; "
                 "action/service endpoints disabled.";
        return;
    }

    PlannerServer* self = this;

    to_pose_server_ =
        std::make_shared<ToPoseServer>(
            node_, kComputePathToPoseActionName,
            [self]() { self->ComputePlan(); });

    through_poses_server_ =
        std::make_shared<ThroughPosesServer>(
            node_, kComputePathThroughPosesActionName,
            [self]() { self->ComputePlanThroughPoses(); });

    path_valid_service_ =
        node_->CreateService<PathValidRequest, PathValidResponse>(
            kIsPathValidServiceName,
            [self](const std::shared_ptr<PathValidRequest>& request,
                   std::shared_ptr<PathValidResponse>& response) {
                const auto path =
                    commsgs::planning_msgs::FromProto(request->path());
                const uint8_t max_cost = request->max_cost() > 0
                                             ? static_cast<uint8_t>(request->max_cost())
                                             : 253;
                response->set_is_valid(self->IsPathValid(
                    path, max_cost, request->consider_unknown_as_obstacle()));
            });

    AINFO << "PlannerServer autolink action/service endpoints started.";
}

PlannerServer::~PlannerServer() {
    if (to_pose_server_) {
        to_pose_server_->Deactivate();
        to_pose_server_.reset();
    }
    if (through_poses_server_) {
        through_poses_server_->Deactivate();
        through_poses_server_.reset();
    }
    path_valid_service_.reset();
    node_.reset();

    for (auto& entry : planners_) {
        entry.second->Deactivate();
        entry.second->Cleanup();
    }

    if (costmap_wrapper_) {
        costmap_wrapper_->Stop();
    }
}

void PlannerServer::LoadPlannerPlugins() {
    if (plugins_loaded_) {
        return;
    }

    planner_ids_.clear();
    planners_.clear();
    planner_ids_concat_.clear();

    std::vector<std::string> plugin_entries;
    if (options_.planner_plugins_size() > 0) {
        plugin_entries.reserve(
            static_cast<size_t>(options_.planner_plugins_size()));
        for (const auto& entry : options_.planner_plugins()) {
            plugin_entries.push_back(entry);
        }
    } else {
        plugin_entries = {"navfn_planner", "dijkstra_planner"};
    }

    LoadExternalPlugins(options_);

    const auto specs = ParsePluginSpecs(plugin_entries);

    for (const auto& spec : specs) {
        if (planners_.count(spec.id) > 0) {
            AWARN << "Duplicate planner plugin id ignored: " << spec.id;
            continue;
        }
        if (!IsPlannerClassRegistered(spec.type)) {
            AFATAL << "Unknown planner plugin type: " << spec.type
                   << " for id: " << spec.id;
            return;
        }

        auto planner_instance = CreatePlannerInstance(spec.type);
        if (!planner_instance) {
            AFATAL << "Failed to create planner plugin: " << spec.id;
            return;
        }

        if (!planner_instance->Configure(options_, spec.id, costmap_wrapper_)) {
            AFATAL << "Failed to configure planner plugin: " << spec.id;
            return;
        }

        planners_.insert({spec.id, planner_instance});
        planner_ids_.push_back(spec.id);
        if (!planner_ids_concat_.empty()) {
            planner_ids_concat_ += ", ";
        }
        planner_ids_concat_ += spec.id;
        AINFO << "Created planner plugin: " << spec.id
              << " (type = " << spec.type << ")";
    }

    if (planners_.empty()) {
        AFATAL << "No planner plugins loaded";
        return;
    }

    if (default_planner_id_.empty() ||
        planners_.find(default_planner_id_) == planners_.end()) {
        default_planner_id_ = planner_ids_.front();
    }

    plugins_loaded_ = true;
    AINFO << "PlannerServer has " << planners_.size()
          << " planners available: " << planner_ids_concat_
          << " (default: " << default_planner_id_ << ")";
}

void PlannerServer::SetPathUpdateCallback(PathUpdateCallback callback) {
    path_update_callback_ = std::move(callback);
}

void PlannerServer::PublishPlan(const commsgs::planning_msgs::Path& path) {
    if (path_update_callback_) {
        path_update_callback_(path);
    }
}

commsgs::planning_msgs::Path PlannerServer::GetPlan(
    const commsgs::geometry_msgs::PoseStamped& start,
    const commsgs::geometry_msgs::PoseStamped& goal,
    const std::string& planner_id, std::function<bool()> cancel_checker) {
    commsgs::planning_msgs::Path path;
    AINFO << "Planning algorithm " << planner_id
          << " is trying to find a path from (" << start.pose.position.x << ", "
          << start.pose.position.y << ")"
          << " to (" << goal.pose.position.x << "," << goal.pose.position.y
          << ")";

    uint32_t return_code = 0;
    std::string resolved_planner_id = planner_id;
    if (planners_.find(planner_id) != planners_.end()) {
        return_code = planners_[planner_id]->CreatePlan(start, goal, path,
                                                        cancel_checker);
    } else if (planners_.size() == 1 && planner_id.empty()) {
        resolved_planner_id = planners_.begin()->first;
        AWARN << "No planner specified; using " << resolved_planner_id;
        return_code = planners_[resolved_planner_id]->CreatePlan(
            start, goal, path, cancel_checker);
    } else {
        throw common::InvalidPlanner("Planner id " + planner_id +
                                     " is invalid. Available: " +
                                     planner_ids_concat_);
    }

    ThrowOnPlannerResultCode(return_code, resolved_planner_id);
    return path;
}

void PlannerServer::WaitForCostmap() {
    const double timeout_sec = options_.costmap_update_timeout();
    const auto waiting_start = std::chrono::steady_clock::now();

    while (costmap_wrapper_ && !costmap_wrapper_->isCurrent()) {
        if (timeout_sec > 0.0) {
            const auto elapsed =
                std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                              waiting_start)
                    .count();
            if (elapsed > timeout_sec) {
                throw common::PlannerTimedOut(
                    "Costmap timed out waiting for update");
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool PlannerServer::TransformPosesToGlobalFrame(
    commsgs::geometry_msgs::PoseStamped& curr_start,
    commsgs::geometry_msgs::PoseStamped& curr_goal) {
    if (!costmap_wrapper_) {
        return false;
    }

    NormalizePoseFrame(curr_start, costmap_wrapper_->getGlobalFrameID());
    NormalizePoseFrame(curr_goal, costmap_wrapper_->getGlobalFrameID());

    commsgs::geometry_msgs::PoseStamped transformed_start;
    commsgs::geometry_msgs::PoseStamped transformed_goal;
    if (!costmap_wrapper_->transformPoseToGlobalFrame(curr_start,
                                                      transformed_start) ||
        !costmap_wrapper_->transformPoseToGlobalFrame(curr_goal,
                                                      transformed_goal)) {
        return false;
    }

    curr_start = transformed_start;
    curr_goal = transformed_goal;
    return true;
}

bool PlannerServer::ValidatePath(
    const commsgs::geometry_msgs::PoseStamped& curr_goal,
    const commsgs::planning_msgs::Path& path, const std::string& planner_id) {
    if (path.poses.empty()) {
        AWARN << "Planning algorithm " << planner_id
              << " failed to generate a valid path to ("
              << curr_goal.pose.position.x << ", " << curr_goal.pose.position.y
              << ")";
        return false;
    }

    AINFO << "Found valid path of size " << path.poses.size() << " to ("
          << curr_goal.pose.position.x << ", " << curr_goal.pose.position.y
          << ")";
    return true;
}

void PlannerServer::ComputePlan() {
    std::lock_guard<std::mutex> lock(dynamic_params_mutex_);

    auto& server = to_pose_server_;
    if (!server || !server->IsServerActive() || server->IsCancelRequested()) {
        return;
    }

    auto goal = server->GetCurrentGoal();
    if (!goal) {
        return;
    }
    if (server->IsPreemptRequested()) {
        goal = server->AcceptPendingGoal();
    }
    if (!goal || !server->IsServerActive()) {
        return;
    }

    auto result = std::make_shared<task_proto::ComputePathToPoseAction_Result>();
    const auto start_time = Time::Now();
    metrics_.plans_requested.fetch_add(1, std::memory_order_relaxed);

    commsgs::geometry_msgs::PoseStamped start;
    commsgs::geometry_msgs::PoseStamped goal_pose;

    try {
        if (server->IsCancelRequested()) {
            return;
        }

        WaitForCostmap();

        if (goal->use_start() && goal->has_start()) {
            start = commsgs::geometry_msgs::FromProto(goal->start());
        } else if (!costmap_wrapper_ ||
                   !costmap_wrapper_->getRobotPose(start)) {
            throw common::PlannerTFError("Unable to get start pose");
        }

        goal_pose = commsgs::geometry_msgs::FromProto(goal->goal());
        if (!TransformPosesToGlobalFrame(start, goal_pose)) {
            throw common::PlannerTFError(
                "Unable to transform poses to global frame");
        }

        auto cancel_checker = [&]() { return server->IsCancelRequested(); };
        const auto path =
            GetPlan(start, goal_pose, goal->planner_id(), cancel_checker);
        if (!ValidatePath(goal_pose, path, goal->planner_id())) {
            throw common::NoValidPathCouldBeFound(goal->planner_id() +
                                                  " generated an empty path");
        }

        PublishPlan(path);
        metrics_.plans_succeeded.fetch_add(1, std::memory_order_relaxed);

        *result->mutable_path() = commsgs::planning_msgs::ToProto(path);
        *result->mutable_planning_time() =
            commsgs::builtin_interfaces::ToProto(Time::Now() - start_time);
        result->set_error_code(task_proto::COMPUTE_PATH_TO_POSE_NONE);

        const auto cycle_duration = Time::Now() - start_time;
        if (max_planner_duration_ > 0.0 &&
            cycle_duration.Seconds() > max_planner_duration_) {
            AWARN << "Planner loop missed its desired rate of "
                  << (1.0 / max_planner_duration_) << " Hz. Current loop rate is "
                  << (1.0 / cycle_duration.Seconds()) << " Hz";
        }

        server->SucceededCurrent(result);
    } catch (const common::PlannerCancelled&) {
        metrics_.plans_failed.fetch_add(1, std::memory_order_relaxed);
        result->set_error_msg("Goal was canceled. Canceling planning action.");
        server->TerminateAll(result);
    } catch (const std::exception& ex) {
        metrics_.plans_failed.fetch_add(1, std::memory_order_relaxed);
        result->set_error_code(MapToComputePathToPoseErrorCode(ex));
        result->set_error_msg(ex.what());
        server->TerminateCurrent(result);
    }
}

void PlannerServer::ComputePlanThroughPoses() {
    std::lock_guard<std::mutex> lock(dynamic_params_mutex_);

    auto& server = through_poses_server_;
    if (!server || !server->IsServerActive() || server->IsCancelRequested()) {
        return;
    }

    auto goal = server->GetCurrentGoal();
    if (!goal) {
        return;
    }
    if (server->IsPreemptRequested()) {
        goal = server->AcceptPendingGoal();
    }
    if (!goal || !server->IsServerActive()) {
        return;
    }

    auto result =
        std::make_shared<task_proto::ComputePathThroughPosesAction_Result>();
    const auto start_time = Time::Now();
    metrics_.plans_requested.fetch_add(1, std::memory_order_relaxed);

    commsgs::geometry_msgs::PoseStamped curr_start;
    commsgs::geometry_msgs::PoseStamped curr_goal;

    try {
        if (server->IsCancelRequested()) {
            return;
        }

        WaitForCostmap();

        std::vector<commsgs::geometry_msgs::PoseStamped> goals;
        if (goal->has_goals()) {
            goals.reserve(static_cast<size_t>(goal->goals().goals_size()));
            for (const auto& g : goal->goals().goals()) {
                goals.push_back(commsgs::geometry_msgs::FromProto(g));
            }
        }
        if (goals.empty()) {
            throw common::NoViapointsGiven("No viapoints given");
        }

        if (goal->use_start() && goal->has_start()) {
            curr_start = commsgs::geometry_msgs::FromProto(goal->start());
        } else if (!costmap_wrapper_ ||
                   !costmap_wrapper_->getRobotPose(curr_start)) {
            throw common::PlannerTFError("Unable to get start pose");
        }

        auto cancel_checker = [&]() { return server->IsCancelRequested(); };

        commsgs::planning_msgs::Path merged_path;
        for (size_t i = 0; i < goals.size(); ++i) {
            if (server->IsCancelRequested()) {
                throw common::PlannerCancelled("ComputePathThroughPoses cancelled");
            }

            commsgs::geometry_msgs::PoseStamped segment_start;
            if (i == 0) {
                segment_start = curr_start;
            } else {
                segment_start = merged_path.poses.back();
                segment_start.header = merged_path.header;
            }
            curr_goal = goals[i];

            if (!TransformPosesToGlobalFrame(segment_start, curr_goal)) {
                throw common::PlannerTFError(
                    "Unable to transform poses to global frame");
            }

            auto segment =
                GetPlan(segment_start, curr_goal, goal->planner_id(), cancel_checker);
            if (!ValidatePath(curr_goal, segment, goal->planner_id())) {
                throw common::NoValidPathCouldBeFound(goal->planner_id() +
                                                      " generated an empty path");
            }

            merged_path.poses.insert(merged_path.poses.end(),
                                     segment.poses.begin(), segment.poses.end());
            merged_path.header = segment.header;
        }

        PublishPlan(merged_path);
        metrics_.plans_succeeded.fetch_add(1, std::memory_order_relaxed);

        *result->mutable_path() = commsgs::planning_msgs::ToProto(merged_path);
        *result->mutable_planning_time() =
            commsgs::builtin_interfaces::ToProto(Time::Now() - start_time);
        result->set_error_code(task_proto::COMPUTE_PATH_THROUGH_POSES_NONE);

        const auto cycle_duration = Time::Now() - start_time;
        if (max_planner_duration_ > 0.0 &&
            cycle_duration.Seconds() > max_planner_duration_) {
            AWARN << "Planner loop missed its desired rate of "
                  << (1.0 / max_planner_duration_) << " Hz. Current loop rate is "
                  << (1.0 / cycle_duration.Seconds()) << " Hz";
        }

        server->SucceededCurrent(result);
    } catch (const common::PlannerCancelled&) {
        metrics_.plans_failed.fetch_add(1, std::memory_order_relaxed);
        result->set_error_msg("Goal was canceled. Canceling planning action.");
        server->TerminateAll(result);
    } catch (const std::exception& ex) {
        metrics_.plans_failed.fetch_add(1, std::memory_order_relaxed);
        result->set_error_code(MapToComputePathThroughPosesErrorCode(ex));
        result->set_error_msg(ex.what());
        server->TerminateCurrent(result);
    }
}

bool PlannerServer::IsPathValid(const commsgs::planning_msgs::Path& path,
                                uint8_t max_cost,
                                bool consider_unknown_as_obstacle) const {
    if (path.poses.empty()) {
        return false;
    }
    if (!costmap_wrapper_ || !costmap_) {
        return false;
    }

    commsgs::geometry_msgs::PoseStamped current_pose;
    unsigned int closest_point_index = 0;
    if (!costmap_wrapper_->getRobotPose(current_pose)) {
        return true;
    }

    float closest_distance = std::numeric_limits<float>::max();
    const auto& current_point = current_pose.pose.position;
    for (size_t i = 0; i < path.poses.size(); ++i) {
        const float distance = static_cast<float>(utils::EuclideanDistance(
            current_point, path.poses[i].pose.position));
        if (distance < closest_distance) {
            closest_point_index = static_cast<unsigned int>(i);
            closest_distance = distance;
        }
    }

    std::unique_lock<map::costmap_2d::Costmap2D::mutex_t> lock(
        *(costmap_->getMutex()));

    map::costmap_2d::FootprintCollisionChecker<map::costmap_2d::Costmap2D*>
        collision_checker(costmap_);
    const bool use_radius = costmap_wrapper_->getUseRadius();
    const auto footprint = costmap_wrapper_->getRobotFootprint();

    unsigned int mx = 0;
    unsigned int my = 0;
    unsigned int cost = map::costmap_2d::FREE_SPACE;

    for (size_t i = closest_point_index; i < path.poses.size(); ++i) {
        const auto& position = path.poses[i].pose.position;
        if (use_radius) {
            if (costmap_->worldToMap(position.x, position.y, mx, my)) {
                cost = costmap_->getCost(mx, my);
            } else {
                cost = map::costmap_2d::LETHAL_OBSTACLE;
            }
        } else {
            const double theta =
                transform::tf2::getYaw(path.poses[i].pose.orientation);
            cost = static_cast<unsigned int>(collision_checker.footprintCostAtPose(
                position.x, position.y, theta, footprint));
        }

        if (cost == map::costmap_2d::NO_INFORMATION &&
            consider_unknown_as_obstacle) {
            cost = map::costmap_2d::LETHAL_OBSTACLE;
        } else if (cost == map::costmap_2d::NO_INFORMATION) {
            cost = map::costmap_2d::FREE_SPACE;
        }

        if (use_radius &&
            (cost >= max_cost ||
             cost == map::costmap_2d::LETHAL_OBSTACLE ||
             cost == map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) {
            return false;
        }
        if (!use_radius &&
            (cost == map::costmap_2d::LETHAL_OBSTACLE || cost >= max_cost)) {
            return false;
        }
    }

    return true;
}

}  // namespace planning
}  // namespace autonomy

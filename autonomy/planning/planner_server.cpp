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

#include "autonomy/common/string_util.hpp"
#include <unistd.h>  // for getpid()

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <future>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "autonomy/common/config.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/common/time.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/map/costmap_2d/utils/occ_grid_values.hpp"
#include "autonomy/planning/common/planner_exceptions.hpp"
#include "autonomy/planning/constants.hpp"
#include "autonomy/planning/plugin_manager.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"
#include "autonomy/planning/smoother_server.hpp"
#include "autonomy/planning/utils/path_simplifier.hpp"

namespace autonomy {
namespace planning {

using Time = commsgs::builtin_interfaces::Time;

namespace {

bool IsCellBlocked(unsigned char cost, bool allow_unknown) {
    if (cost == map::costmap_2d::LETHAL_OBSTACLE ||
        cost == map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return true;
    }
    if (!allow_unknown && cost == map::costmap_2d::NO_INFORMATION) {
        return true;
    }
    return false;
}

bool IsCostmapReady(map::costmap_2d::Costmap2DWrapper* wrapper) {
    return wrapper != nullptr && (wrapper->isReady() || wrapper->isCurrent());
}

int64_t SteadyNowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

/** Empty frame_id is treated as the costmap global frame (see planning contract). */
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

void AppendPathSegment(commsgs::planning_msgs::Path& merged,
                       const commsgs::planning_msgs::Path& segment) {
    if (segment.poses.empty()) {
        return;
    }
    if (merged.poses.empty()) {
        merged = segment;
        return;
    }

    merged.header = segment.header;
    size_t start_idx = 0;
    const auto& last = merged.poses.back();
    const auto& first = segment.poses.front();
    const double dx = last.pose.position.x - first.pose.position.x;
    const double dy = last.pose.position.y - first.pose.position.y;
    if ((dx * dx + dy * dy) < 1e-8) {
        start_idx = 1;
    }
    merged.poses.insert(merged.poses.end(),
                        segment.poses.begin() +
                            static_cast<std::ptrdiff_t>(start_idx),
                        segment.poses.end());
}

}  // namespace

PlannerServer::PlannerServer(const proto::PlannerOptions& options)
    : options_{options} {
    costmap_wrapper_ = std::make_shared<map::costmap_2d::Costmap2DWrapper>(
        options_.costmap(), kCostmapTopicName);
    if (!costmap_wrapper_) {
        AFATAL << "Failed to configure costmap wrapper. costmap_wrapper is "
                  "nullptr";
        return;
    }

    costmap_ = costmap_wrapper_->getCostmap();

    default_planner_id_ = options_.default_planner_id().empty()
                                ? "navfn_planner"
                                : options_.default_planner_id();

    LoadPlugins();

    // 处理 expected_planner_frequency
    double expected_planner_frequency = options_.expected_planner_frequency();
    if (expected_planner_frequency > 0) {
        max_planner_duration_ = 1 / expected_planner_frequency;
    } else {
        AWARN
            << "The expected planner frequency parameter is "
            << expected_planner_frequency
            << " Hz. The value should be greater than 0.0 to turn on duration "
               "overrun warning messages";
        max_planner_duration_ = 0.0;
    }
}

void PlannerServer::LoadPlugins() {
    if (plugins_loaded_) {
        return;
    }

    planner_ids_.clear();
    planner_types_.clear();
    planners_.clear();
    planner_ids_concat_.clear();

    std::vector<std::string> plugin_entries;
    if (options_.planner_plugins_size() > 0) {
        plugin_entries.reserve(static_cast<size_t>(options_.planner_plugins_size()));
        for (const auto& entry : options_.planner_plugins()) {
            plugin_entries.push_back(entry);
        }
    } else {
        plugin_entries = {"navfn_planner", "dijkstra_planner"};
    }

    auto& loader = PluginManager::Instance();
    loader.Initialize(options_);

    const auto specs =
        PluginManager::ParsePlannerPluginEntries(plugin_entries);

    for (const auto& spec : specs) {
        if (planners_.find(spec.id) != planners_.end()) {
            AWARN << "Duplicate planner plugin id ignored: " << spec.id;
            continue;
        }
        if (!loader.IsPlannerRegistered(spec.type)) {
            AFATAL << "Unknown planner plugin type: " << spec.type
                   << " for id: " << spec.id;
            return;
        }

        auto planner_instance = loader.CreatePlanner(spec.type);
        if (!planner_instance) {
            AFATAL << "Failed to create planner plugin: " << spec.id
                   << " type: " << spec.type;
            return;
        }

        if (!planner_instance->Configure(options_, spec.id, costmap_wrapper_)) {
            AFATAL << "Failed to configure planner plugin: " << spec.id;
            return;
        }

        planners_.insert({spec.id, planner_instance});
        planner_ids_.push_back(spec.id);
        planner_types_.push_back(spec.type);
        if (!planner_ids_concat_.empty()) {
            planner_ids_concat_ += ", ";
        }
        planner_ids_concat_ += spec.id;

        AINFO << "Created planner plugin: " << spec.id << " (type = " << spec.type << ")";
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

    AINFO << "Planner Server has " << planners_.size()
          << " planners available: " << planner_ids_concat_
          << " (default: " << default_planner_id_ << ")";
}

void PlannerServer::Start() {
    if (!costmap_wrapper_) {
        AFATAL << "Costmap wrapper is null";
        return;
    }

    if (shutdown_called_.exchange(false)) {
        for (auto& entry : planners_) {
            entry.second->Configure(options_, entry.first, costmap_wrapper_);
        }
    }

    costmap_wrapper_->Start();

    for (auto it = planners_.begin(); it != planners_.end(); ++it) {
        it->second->Activate();
    }
}

void PlannerServer::Shutdown() {
    if (shutdown_called_.exchange(true)) {
        return;
    }

    for (auto& entry : planners_) {
        entry.second->Deactivate();
        entry.second->Cleanup();
    }

    if (costmap_wrapper_) {
        costmap_wrapper_->Stop();
    }
}

void PlannerServer::ReconfigurePlugins() {
    if (!costmap_wrapper_) {
        return;
    }
    for (auto& entry : planners_) {
        entry.second->Configure(options_, entry.first, costmap_wrapper_);
    }
}

void PlannerServer::SetSmootherServer(
    const std::shared_ptr<SmootherServer>& smoother) {
    smoother_server_ = smoother;
}

void PlannerServer::SetPathUpdateCallback(PathUpdateCallback callback) {
    path_update_callback_ = std::move(callback);
}

bool PlannerServer::AllowUnknownForValidation(
    const std::string& planner_id) const {
    const std::string resolved =
        planner_id.empty() ? default_planner_id_ : planner_id;
    if (resolved == "dijkstra_planner") {
        return options_.dijkstra().allow_unknown();
    }
    if (resolved == "theta_star_planner") {
        return options_.theta_star().allow_unknown();
    }
    return options_.navfn().allow_unknown();
}

commsgs::planning_msgs::Path PlannerServer::PostProcessPath(
    commsgs::planning_msgs::Path path,
    const std::function<bool()>& cancel_checker) {
    if (options_.path_simplify_epsilon() > 0.0) {
        path = utils::SimplifyPath(path, options_.path_simplify_epsilon());
    }

    if (options_.auto_smooth_after_plan()) {
        if (auto smoother = smoother_server_.lock()) {
            const double duration_sec = options_.auto_smooth_duration() > 0.0
                                            ? options_.auto_smooth_duration()
                                            : 1.0;
            const auto max_time = std::chrono::milliseconds(
                static_cast<int>(duration_sec * 1000.0));
            path = smoother
                       ->SmoothPath(path, smoother->GetDefaultSmootherId(),
                                    max_time, false, cancel_checker)
                       .path;
        }
    }
    return path;
}

commsgs::planning_msgs::Path PlannerServer::GetPlan(
    const commsgs::geometry_msgs::PoseStamped& start,
    const commsgs::geometry_msgs::PoseStamped& goal,
    const std::string& planner_id, std::function<bool()> cancel_checker) {
    commsgs::planning_msgs::Path path;
    AINFO << "Planning algorithm " << planner_id
          << " is trying to find a path from (" << start.pose.position.x << ", "
          << start.pose.position.y << ")"
          << " to "
          << "(" << goal.pose.position.x << "," << goal.pose.position.y << ")";

    uint32_t return_code = 0;
    std::string resolved_planner_id = planner_id;
    if (planners_.find(planner_id) != planners_.end()) {
        return_code = planners_[planner_id]->CreatePlan(start, goal, path,
                                                        cancel_checker);
    } else {
        if (planners_.size() == 1 && planner_id.empty()) {
            resolved_planner_id = planners_.begin()->first;
            AWARN
                << "No planners specified in action call. Server will use only "
                   "plugin "
                << planner_ids_concat_
                << " in server. This warning will appear once.";
            return_code = planners_[resolved_planner_id]->CreatePlan(
                start, goal, path, cancel_checker);
        } else {
            AERROR << "planner " << planner_id << " is not a valid planner. "
                   << "Planner names are: " << planner_ids_concat_;
            throw common::InvalidPlanner("Planner id " + planner_id +
                                         " is invalid");
        }
    }

    ThrowOnPlannerResultCode(return_code, resolved_planner_id);

    return path;
}

void PlannerServer::WaitForCostmap() {
    constexpr int64_t kReadyCacheMs = 800;
    const int64_t now_ms = SteadyNowMs();
    const int64_t last_ready_ms =
        last_costmap_ready_time_ms_.load(std::memory_order_acquire);
    if (last_ready_ms > 0 && (now_ms - last_ready_ms) <= kReadyCacheMs) {
        return;
    }

    // 如果未配置超时时间，则一直等待直到 costmap 当前
    const double timeout_sec = options_.costmap_update_timeout();
    const bool use_timeout = timeout_sec > 0.0;

    bool waited = false;

    const auto start_time = std::chrono::steady_clock::now();

    while (true) {
        if (IsCostmapReady(costmap_wrapper_.get())) {
            costmap_received_.store(true, std::memory_order_release);
            last_costmap_ready_time_ms_.store(
                SteadyNowMs(), std::memory_order_release);
            break;
        }

        if (!waited) {
            AINFO << "Waiting for global map (Costmap2D) to become ready"
                  << (use_timeout ? ::autonomy::common::StrCat(
                                        " (timeout = ", timeout_sec, " s)")
                                  : " (no timeout)");
            waited = true;
        }

        if (use_timeout) {
            const auto now = std::chrono::steady_clock::now();
            const auto elapsed_duration =
                std::chrono::duration_cast<std::chrono::duration<double>>(
                    now - start_time);
            const double elapsed = elapsed_duration.count();
            if (elapsed > timeout_sec) {
                AWARN << "WaitForCostmap timeout: global map is still not "
                         "confirmed "
                         "ready after "
                      << elapsed << " seconds.";
                break;
            }
        }

        // 以较小频率轮询，避免忙等
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

bool PlannerServer::TransformPoseToGlobalFrame(
    commsgs::geometry_msgs::PoseStamped& pose) {
    if (!costmap_wrapper_) {
        AERROR << "Costmap wrapper is null, cannot transform pose";
        return false;
    }

    NormalizePoseFrame(pose, costmap_wrapper_->getGlobalFrameID());

    commsgs::geometry_msgs::PoseStamped transformed_pose;
    if (!costmap_wrapper_->transformPoseToGlobalFrame(pose, transformed_pose)) {
        AERROR << "Failed to transform pose to global frame";
        return false;
    }

    pose = transformed_pose;
    return true;
}

bool PlannerServer::TransformPosesToGlobalFrame(
    commsgs::geometry_msgs::PoseStamped& curr_start,
    commsgs::geometry_msgs::PoseStamped& curr_goal) {
    if (!costmap_wrapper_) {
        AERROR << "Costmap wrapper is null, cannot transform poses";
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
        AERROR << "Failed to transform poses to global frame";
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
    if (path_update_callback_) {
        path_update_callback_(path);
    }
    return true;
}

void PlannerServer::ValidateStartGoalOnCostmap(
    const commsgs::geometry_msgs::PoseStamped& start,
    const commsgs::geometry_msgs::PoseStamped& goal,
    const std::string& planner_id) {
    if (!costmap_) {
        throw common::PlannerException("Costmap is null");
    }

    std::unique_lock<map::costmap_2d::Costmap2D::mutex_t> lock(
        *(costmap_->getMutex()));

    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx,
                              my)) {
        throw common::StartOutsideMapBounds("Start is outside map bounds");
    }
    const bool allow_unknown = AllowUnknownForValidation(planner_id);
    const unsigned char start_cost = costmap_->getCost(mx, my);
    if (IsCellBlocked(start_cost, allow_unknown)) {
        throw common::StartOccupied("Start is occupied");
    }

    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx,
                              my)) {
        throw common::GoalOutsideMapBounds("Goal is outside map bounds");
    }
    const unsigned char goal_cost = costmap_->getCost(mx, my);
    if (IsCellBlocked(goal_cost, allow_unknown)) {
        throw common::GoalOccupied("Goal is occupied");
    }
}

commsgs::planning_msgs::Path PlannerServer::ComputePathToPose(
    const commsgs::geometry_msgs::PoseStamped& start,
    const commsgs::geometry_msgs::PoseStamped& goal,
    const std::string& planner_id, std::function<bool()> cancel_checker) {
    if (cancel_checker && cancel_checker()) {
        throw common::PlannerCancelled("ComputePathToPose cancelled");
    }

    const auto start_time = Time::Now();

    metrics_.plans_requested.fetch_add(1, std::memory_order_relaxed);

    try {
        WaitForCostmap();
        if (!IsCostmapReady(costmap_wrapper_.get())) {
            throw common::PlannerTimedOut("Costmap timed out waiting for update");
        }

        commsgs::geometry_msgs::PoseStamped curr_start = start;
        commsgs::geometry_msgs::PoseStamped curr_goal = goal;

        if (!TransformPosesToGlobalFrame(curr_start, curr_goal)) {
            throw common::PlannerTFError(
                "Unable to transform poses to global frame");
        }

        ValidateStartGoalOnCostmap(curr_start, curr_goal, planner_id);

        auto path = GetPlan(curr_start, curr_goal, planner_id, cancel_checker);

        if (!ValidatePath(curr_goal, path, planner_id)) {
            throw common::NoValidPathCouldBeFound(planner_id +
                                                  " generated an empty path");
        }

        path = PostProcessPath(std::move(path), cancel_checker);
        metrics_.plans_succeeded.fetch_add(1, std::memory_order_relaxed);

        const auto cycle_duration = Time::Now() - start_time;
        if (max_planner_duration_ > 0.0 &&
            cycle_duration.Seconds() > max_planner_duration_) {
            AWARN << "Planner loop missed its desired rate of "
                  << (1.0 / max_planner_duration_) << " Hz. Current loop rate is "
                  << (1.0 / cycle_duration.Seconds()) << " Hz";
        }

        return path;
    } catch (...) {
        metrics_.plans_failed.fetch_add(1, std::memory_order_relaxed);
        throw;
    }
}

commsgs::planning_msgs::Path PlannerServer::ComputePathThroughPoses(
    const commsgs::geometry_msgs::PoseStamped& start,
    const std::vector<commsgs::geometry_msgs::PoseStamped>& goals,
    const std::string& planner_id, std::function<bool()> cancel_checker) {
    if (cancel_checker && cancel_checker()) {
        throw common::PlannerCancelled("ComputePathThroughPoses cancelled");
    }
    if (goals.empty()) {
        throw common::NoViapointsGiven("No viapoints given for planning");
    }

    metrics_.plans_requested.fetch_add(1, std::memory_order_relaxed);

    try {
        const auto planning_start_time = Time::Now();

        WaitForCostmap();
        if (!IsCostmapReady(costmap_wrapper_.get())) {
            throw common::PlannerTimedOut("Costmap timed out waiting for update");
        }

        commsgs::geometry_msgs::PoseStamped curr_start = start;
        if (!TransformPoseToGlobalFrame(curr_start)) {
            throw common::PlannerTFError(
                "Unable to transform start pose to global frame");
        }

        commsgs::planning_msgs::Path merged_path;
        for (const auto& goal : goals) {
            if (cancel_checker && cancel_checker()) {
                throw common::PlannerCancelled("ComputePathThroughPoses cancelled");
            }

            commsgs::geometry_msgs::PoseStamped curr_goal = goal;
            if (!TransformPoseToGlobalFrame(curr_goal)) {
                throw common::PlannerTFError(
                    "Unable to transform goal pose to global frame");
            }

            ValidateStartGoalOnCostmap(curr_start, curr_goal, planner_id);

            auto segment =
                GetPlan(curr_start, curr_goal, planner_id, cancel_checker);
            if (!ValidatePath(curr_goal, segment, planner_id)) {
                throw common::NoValidPathCouldBeFound(planner_id +
                                                      " generated an empty path");
            }

            AppendPathSegment(merged_path, segment);
            curr_start = curr_goal;
        }

        merged_path = PostProcessPath(std::move(merged_path), cancel_checker);
        metrics_.plans_succeeded.fetch_add(1, std::memory_order_relaxed);

        const auto cycle_duration = Time::Now() - planning_start_time;
        if (max_planner_duration_ > 0.0 &&
            cycle_duration.Seconds() > max_planner_duration_) {
            AWARN << "Planner loop missed its desired rate of "
                  << (1.0 / max_planner_duration_) << " Hz. Current loop rate is "
                  << (1.0 / cycle_duration.Seconds()) << " Hz";
        }

        return merged_path;
    } catch (...) {
        metrics_.plans_failed.fetch_add(1, std::memory_order_relaxed);
        throw;
    }
}

}  // namespace planning
}  // namespace autonomy
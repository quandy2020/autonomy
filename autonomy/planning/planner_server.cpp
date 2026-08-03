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
#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/planning_msgs/planning_msgs.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/footprint_collision_checker.hpp"
#include "autonomy/planning/common/planner_exceptions.hpp"
#include "autonomy/planning/common/smoother_exceptions.hpp"
#include "autonomy/planning/constants.hpp"
#include "autonomy/planning/utils/simple_smoother.hpp"
#include "autonomy/planning/planner/dijkstra/dijkstra_planner.hpp"
#include "autonomy/planning/planner/navfn/navfn_planner.hpp"
#include "autonomy/planning/planner/theta_star/theta_star_planner.hpp"
#include "autonomy/planning/utils/geometry_utils.hpp"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace planning {

using Time = automsgs::msgs::builtin_interfaces::Time;

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
    const std::string& type, const proto::PlannerOptions& options,
    const std::string& id,
    const std::shared_ptr<map::costmap_2d::Costmap2DWrapper>& costmap) {
    const std::string resolved = ResolvePlannerClass(type);
    if (resolved == "NavfnPlanner") {
        return std::make_shared<planner::navfn::NavfnPlanner>(options, id,
                                                              costmap);
    }
    if (resolved == "DijkstraPlanner") {
        return std::make_shared<planner::dijkstra::DijkstraPlanner>(
            options, id, costmap);
    }
    if (resolved == "ThetaStarPlanner") {
        return std::make_shared<planner::theta_star::ThetaStarPlanner>(
            options, id, costmap);
    }
    auto instance = PluginPm::Instance()->CreateInstance<common::GlobalPlanner>(
        resolved);
    if (!instance) {
        return nullptr;
    }
    AWARN << "Planner plugin " << resolved
          << " created via plugin manager without constructor options; "
             "ensure it self-configures.";
    return common::GlobalPlanner::SharedPtr(std::move(instance));
}

void NormalizePoseFrame(automsgs::msgs::geometry_msgs::PoseStamped& pose,
                          const std::string& global_frame) {
    if (pose.header().frame_id().empty()) {
        AWARN << "Pose has empty frame_id; assuming costmap global frame '"
              << global_frame << "'";
        pose.mutable_header()->set_frame_id(global_frame);
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

    node_ = autolink::CreateNode(kPlannerServerNodeName);
    if (!node_) {
        AWARN << "PlannerServer: autolink node creation failed; "
                 "action/service endpoints disabled.";
        return;
    }

    default_smoother_id_ = options_.default_smoother_id().empty()
                               ? "simple_smoother"
                               : options_.default_smoother_id();
    default_smoother_ = std::make_shared<utils::SimpleSmoother>(
        default_smoother_id_, costmap_wrapper_, options_.simple_smoother());

    RegisterAutolinkEndpoints();
}

PlannerServer::~PlannerServer() {
    if (compute_path_to_pose_server_) {
        compute_path_to_pose_server_->Deactivate();
        compute_path_to_pose_server_.reset();
    }
    if (compute_path_through_poses_server_) {
        compute_path_through_poses_server_->Deactivate();
        compute_path_through_poses_server_.reset();
    }
    if (smooth_path_server_) {
        smooth_path_server_->Deactivate();
        smooth_path_server_.reset();
    }
    path_valid_service_.reset();
    node_.reset();

    planners_.clear();

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

        auto planner_instance =
            CreatePlannerInstance(spec.type, options_, spec.id, costmap_wrapper_);
        if (!planner_instance) {
            AFATAL << "Failed to create planner plugin: " << spec.id;
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

void PlannerServer::PublishPlan(const automsgs::msgs::planning_msgs::Path& path) {
    if (path_update_callback_) {
        path_update_callback_(path);
    }
}

automsgs::msgs::planning_msgs::Path PlannerServer::GetPlan(
    const automsgs::msgs::geometry_msgs::PoseStamped& start,
    const automsgs::msgs::geometry_msgs::PoseStamped& goal,
    const std::string& planner_id, std::function<bool()> cancel_checker) {
    automsgs::msgs::planning_msgs::Path path;
    AINFO << "Planning algorithm " << planner_id
          << " is trying to find a path from (" << start.pose().position().x() << ", "
          << start.pose().position().y() << ")"
          << " to (" << goal.pose().position().x() << "," << goal.pose().position().y()
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
                std::chrono::duration<double>(std::chrono::steady_clock::now() - waiting_start)
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
    automsgs::msgs::geometry_msgs::PoseStamped& curr_start,
    automsgs::msgs::geometry_msgs::PoseStamped& curr_goal) {
    if (!costmap_wrapper_) {
        return false;
    }

    NormalizePoseFrame(curr_start, costmap_wrapper_->getGlobalFrameID());
    NormalizePoseFrame(curr_goal, costmap_wrapper_->getGlobalFrameID());

    automsgs::msgs::geometry_msgs::PoseStamped transformed_start;
    automsgs::msgs::geometry_msgs::PoseStamped transformed_goal;
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
    const automsgs::msgs::geometry_msgs::PoseStamped& curr_goal,
    const automsgs::msgs::planning_msgs::Path& path, const std::string& planner_id) {
    if (path.poses().empty()) {
        AWARN << "Planning algorithm " << planner_id
              << " failed to generate a valid path to ("
              << curr_goal.pose().position().x() << ", " << curr_goal.pose().position().y()
              << ")";
        return false;
    }

    AINFO << "Found valid path of size " << path.poses_size() << " to ("
          << curr_goal.pose().position().x() << ", " << curr_goal.pose().position().y()
          << ")";
    return true;
}

bool PlannerServer::IsPathValid(const automsgs::msgs::planning_msgs::Path& path,
                                uint8_t max_cost,
                                bool consider_unknown_as_obstacle) const {
    if (path.poses().empty()) {
        return false;
    }
    if (!costmap_wrapper_ || !costmap_) {
        return false;
    }

    automsgs::msgs::geometry_msgs::PoseStamped current_pose;
    unsigned int closest_point_index = 0;
    if (!costmap_wrapper_->getRobotPose(current_pose)) {
        return true;
    }

    float closest_distance = std::numeric_limits<float>::max();
    const auto& current_point = current_pose.pose().position();
    for (size_t i = 0; i < path.poses_size(); ++i) {
        const float distance = static_cast<float>(utils::EuclideanDistance(
            current_point, path.poses(i).pose().position()));
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

    for (size_t i = closest_point_index; i < path.poses_size(); ++i) {
        const auto& position = path.poses(i).pose().position();
        if (use_radius) {
            if (costmap_->worldToMap(position.x(), position.y(), mx, my)) {
                cost = costmap_->getCost(mx, my);
            } else {
                cost = map::costmap_2d::LETHAL_OBSTACLE;
            }
        } else {
            const double theta =
                transform::tf2::getYaw(path.poses(i).pose().orientation());
            cost = static_cast<unsigned int>(collision_checker.footprintCostAtPose(
                position.x(), position.y(), theta, footprint));
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

void PlannerServer::ClearEntireCostmap()
{
    if (!costmap_wrapper_) {
        AWARN << "PlannerServer: clear costmap ignored (no costmap)";
        return;
    }
    costmap_wrapper_->resetLayers();
    AINFO << "PlannerServer: global costmap cleared";
}

}  // namespace planning
}  // namespace autonomy

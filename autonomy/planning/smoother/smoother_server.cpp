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

#include "autonomy/planning/smoother/smoother_server.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <unordered_map>

#include "autolink/action/simple_action_server.hpp"
#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/planning/common/planner_interface.hpp"
#include "autonomy/planning/common/smoother_exceptions.hpp"
#include "autonomy/planning/smoother/simple_smoother.hpp"
#include "autonomy/tasks/navigators/action_type.hpp"

namespace autonomy {
namespace planning {

namespace {

using PluginPm = autolink::plugin_manager::PluginManager;

constexpr unsigned int kCollisionMaxCost = 253;

struct PluginSpec {
    std::string id;
    std::string type;
};

const std::unordered_map<std::string, std::string>& SmootherClassAliases() {
    static const std::unordered_map<std::string, std::string> kAliases = {
        {"simple_smoother", "SimpleSmoother"},
    };
    return kAliases;
}

std::string ResolveSmootherClass(const std::string& plugin_id) {
    const auto& aliases = SmootherClassAliases();
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
            spec.type = ResolveSmootherClass(entry_str);
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
    pm->RegisterInProcessClass<common::Smoother>("SimpleSmoother");
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

bool IsSmootherClassRegistered(const std::string& type) {
    const std::string resolved = ResolveSmootherClass(type);
    const auto names =
        PluginPm::Instance()->GetDerivedClassNameByBaseClass<
            common::Smoother>();
    return std::find(names.begin(), names.end(), resolved) != names.end();
}

common::Smoother::SharedPtr CreateSmootherInstance(const std::string& type) {
    auto instance = PluginPm::Instance()->CreateInstance<common::Smoother>(
        ResolveSmootherClass(type));
    return instance ? common::Smoother::SharedPtr(std::move(instance))
                    : nullptr;
}

void ApplySmootherOptions(common::Smoother& smoother,
                          const proto::PlannerOptions& options) {
    if (auto* simple = dynamic_cast<smoother::SimpleSmoother*>(&smoother)) {
        simple->ApplyOptions(options.simple_smoother());
    }
}

}  // namespace

SmootherServer::SmootherServer(
    const proto::PlannerOptions& options,
    map::costmap_2d::Costmap2DWrapper::SharedPtr costmap_wrapper)
    : options_(options), costmap_wrapper_(std::move(costmap_wrapper)) {
    default_smoother_id_ = options_.default_smoother_id().empty()
                               ? "simple_smoother"
                               : options_.default_smoother_id();
    LoadPlugins();
}

void SmootherServer::LoadPlugins() {
    if (plugins_loaded_) {
        return;
    }

    smoothers_.clear();
    smoother_ids_concat_.clear();

    std::vector<std::string> plugin_entries;
    if (options_.smoother_plugins_size() > 0) {
        plugin_entries.reserve(
            static_cast<size_t>(options_.smoother_plugins_size()));
        for (const auto& entry : options_.smoother_plugins()) {
            plugin_entries.push_back(entry);
        }
    } else {
        plugin_entries = {"simple_smoother"};
    }

    LoadExternalPlugins(options_);

    const auto specs = ParsePluginSpecs(plugin_entries);

    for (const auto& spec : specs) {
        if (smoothers_.count(spec.id) > 0) {
            AWARN << "Duplicate smoother plugin id ignored: " << spec.id;
            continue;
        }
        if (!IsSmootherClassRegistered(spec.type)) {
            AFATAL << "Unknown smoother plugin type: " << spec.type;
            return;
        }

        auto smoother = CreateSmootherInstance(spec.type);
        if (!smoother) {
            AFATAL << "Failed to create smoother: " << spec.id;
            return;
        }

        ApplySmootherOptions(*smoother, options_);
        smoother->Configure(spec.id, nullptr, costmap_wrapper_);
        smoothers_.insert({spec.id, smoother});
        if (!smoother_ids_concat_.empty()) {
            smoother_ids_concat_ += " ";
        }
        smoother_ids_concat_ += spec.id;
        AINFO << "Created smoother: " << spec.id << ", type: " << spec.type;
    }

    if (smoothers_.empty()) {
        AFATAL << "No smoother plugins loaded";
        return;
    }

    if (default_smoother_id_.empty() ||
        smoothers_.find(default_smoother_id_) == smoothers_.end()) {
        default_smoother_id_ = smoothers_.begin()->first;
    }

    plugins_loaded_ = true;
    AINFO << "SmootherServer has " << smoothers_.size()
          << " smoothers available: " << smoother_ids_concat_
          << " (default: " << default_smoother_id_ << ")";
}

void SmootherServer::Start() {
    if (shutdown_called_.exchange(false)) {
        for (auto& entry : smoothers_) {
            ApplySmootherOptions(*entry.second, options_);
            entry.second->Configure(entry.first, nullptr, costmap_wrapper_);
        }
    }

    for (auto& entry : smoothers_) {
        entry.second->Activate();
    }
}

void SmootherServer::Shutdown() {
    if (shutdown_called_.exchange(true)) {
        return;
    }
    DetachAutolinkNode();
    for (auto& entry : smoothers_) {
        entry.second->Deactivate();
        entry.second->Cleanup();
    }
    AINFO << "SmootherServer shutdown successfully.";
}

bool SmootherServer::IsPathInCollision(
    const commsgs::planning_msgs::Path& path) const {
    if (!costmap_wrapper_ || path.poses.empty()) {
        return true;
    }

    auto* grid = costmap_wrapper_->getCostmap();
    if (!grid) {
        return true;
    }

    for (const auto& pose : path.poses) {
        unsigned int mx = 0;
        unsigned int my = 0;
        if (!grid->worldToMap(pose.pose.position.x, pose.pose.position.y, mx,
                              my)) {
            return true;
        }
        const unsigned char cost = grid->getCost(mx, my);
        if (cost == map::costmap_2d::NO_INFORMATION ||
            cost >= kCollisionMaxCost) {
            return true;
        }
    }
    return false;
}

void SmootherServer::SetPathUpdateCallback(PathUpdateCallback callback) {
    path_update_callback_ = std::move(callback);
}

SmoothPathResult SmootherServer::SmoothPath(
    commsgs::planning_msgs::Path path, const std::string& smoother_id,
    const std::chrono::milliseconds& max_time, bool check_for_collisions,
    const std::function<bool()>& cancel_checker) {
    if (cancel_checker && cancel_checker()) {
        throw common::SmootherTimedOut("SmoothPath cancelled");
    }
    if (path.poses.size() < 2) {
        throw common::InvalidPath("Path must contain at least 2 poses");
    }

    const std::string resolved_id = smoother_id.empty() ? default_smoother_id_
                                                          : smoother_id;

    const auto smoother_it = smoothers_.find(resolved_id);
    if (smoother_it == smoothers_.end()) {
        throw common::InvalidSmoother("Smoother id " + resolved_id +
                                      " is invalid. Available: " +
                                      smoother_ids_concat_);
    }

    const auto start = std::chrono::steady_clock::now();
    const bool was_completed = smoother_it->second->Smooth(path, max_time);
    const auto elapsed = std::chrono::steady_clock::now() - start;
    const double duration_sec =
        std::chrono::duration<double>(elapsed).count();

    if (check_for_collisions && IsPathInCollision(path)) {
        throw common::SmoothedPathInCollision(
            "Smoothed path is in collision with the costmap");
    }

    if (!was_completed) {
        throw common::SmootherTimedOut("Smoother exceeded max duration");
    }

    if (path_update_callback_) {
        path_update_callback_(path);
    }

    return SmoothPathResult{path, was_completed, duration_sec};
}

namespace {

namespace task_proto = tasks::behavior_tree::task_proto;
using SmoothPathServer =
    autolink::action::SimpleActionServer<tasks::behavior_tree::SmoothPathActionTraits>;

std::chrono::milliseconds MaxSmoothingDuration(
    const task_proto::SmoothPathAction_Goal& goal) {
    if (!goal.has_max_smoothing_duration()) {
        return std::chrono::milliseconds(1000);
    }
    const double max_sec = commsgs::builtin_interfaces::FromProto(
                               goal.max_smoothing_duration())
                               .Seconds();
    if (max_sec <= 0.0) {
        return std::chrono::milliseconds(1000);
    }
    return std::chrono::milliseconds(static_cast<int>(max_sec * 1000.0));
}

void HandleSmootherActionException(
    const std::exception& ex,
    const std::shared_ptr<task_proto::SmoothPathAction_Result>& result,
    const std::shared_ptr<SmoothPathServer>& server) {
    result->set_error_msg(ex.what());
    if (dynamic_cast<const common::InvalidSmoother*>(&ex)) {
        result->set_error_code(task_proto::SMOOTH_PATH_INVALID_SMOOTHER);
    } else if (dynamic_cast<const common::SmootherTimedOut*>(&ex)) {
        result->set_error_code(task_proto::SMOOTH_PATH_TIMEOUT);
    } else if (dynamic_cast<const common::SmoothedPathInCollision*>(&ex)) {
        result->set_error_code(task_proto::SMOOTH_PATH_SMOOTHED_PATH_IN_COLLISION);
    } else if (dynamic_cast<const common::FailedToSmoothPath*>(&ex)) {
        result->set_error_code(task_proto::SMOOTH_PATH_FAILED_TO_SMOOTH);
    } else if (dynamic_cast<const common::InvalidPath*>(&ex)) {
        result->set_error_code(task_proto::SMOOTH_PATH_INVALID_PATH);
    } else {
        result->set_error_code(task_proto::SMOOTH_PATH_UNKNOWN);
    }
    AERROR << result->error_msg();
    server->TerminateCurrent(result);
}

}  // namespace

struct SmootherServer::AutolinkActionServers {
    std::shared_ptr<SmoothPathServer> smooth_path;
};

SmootherServer::~SmootherServer() {
    DetachAutolinkNode();
}

bool SmootherServer::AttachAutolinkNode(std::shared_ptr<autolink::Node> node) {
    if (!node) {
        return false;
    }
    if (autolink_actions_) {
        return true;
    }
    autolink_actions_ = new AutolinkActionServers();
    SmootherServer* self = this;

    autolink_actions_->smooth_path = std::make_shared<SmoothPathServer>(
        node, tasks::behavior_tree::kSmoothPathActionName, [self]() {
            auto& server = self->autolink_actions_->smooth_path;
            if (!server || !server->IsServerActive() ||
                server->IsCancelRequested()) {
                return;
            }
            auto goal = server->GetCurrentGoal();
            if (!goal) {
                return;
            }
            if (server->IsPreemptRequested()) {
                goal = server->AcceptPendingGoal();
            }
            if (!goal || !server->IsServerActive() ||
                server->IsCancelRequested()) {
                return;
            }

            auto result =
                std::make_shared<task_proto::SmoothPathAction_Result>();
            const auto start = std::chrono::steady_clock::now();

            try {
                AINFO << "Received a path to smooth.";
                commsgs::planning_msgs::Path path =
                    commsgs::planning_msgs::FromProto(goal->path());
                const auto max_time = MaxSmoothingDuration(*goal);
                auto cancel_checker = [&]() {
                    return server->IsCancelRequested();
                };

                const auto smooth_result = self->SmoothPath(
                    path, goal->smoother_id(), max_time,
                    goal->check_for_collisions(), cancel_checker);

                *result->mutable_path() =
                    commsgs::planning_msgs::ToProto(smooth_result.path);
                result->set_was_completed(smooth_result.was_completed);
                const auto elapsed = std::chrono::steady_clock::now() - start;
                *result->mutable_smoothing_duration() =
                    commsgs::builtin_interfaces::ToProto(
                        commsgs::builtin_interfaces::Duration::FromSeconds(
                            std::chrono::duration<double>(elapsed).count()));
                result->set_error_code(task_proto::SMOOTH_PATH_NONE);
                server->SucceededCurrent(result);
            } catch (const std::exception& ex) {
                HandleSmootherActionException(ex, result, server);
            }
        });

    AINFO << "SmootherServer autolink actions attached.";
    return true;
}

void SmootherServer::DetachAutolinkNode() {
    if (!autolink_actions_) {
        return;
    }
    if (autolink_actions_->smooth_path) {
        autolink_actions_->smooth_path->Deactivate();
    }
    delete autolink_actions_;
    autolink_actions_ = nullptr;
}

}  // namespace planning
}  // namespace autonomy

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

#include "autonomy/planning/smoother_server.hpp"

#include <chrono>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/planning/common/smoother_exceptions.hpp"
#include "autonomy/planning/plugin_manager.hpp"

namespace autonomy {
namespace planning {

namespace {

constexpr unsigned int kCollisionMaxCost = 253;

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
        plugin_entries = {"simple_smoother", "savitzky_golay_smoother"};
    }

    auto& loader = PluginManager::Instance();
    loader.Initialize(options_);

    const auto specs =
        PluginManager::ParseSmootherPluginEntries(plugin_entries);

    for (const auto& spec : specs) {
        if (smoothers_.count(spec.id) > 0) {
            AWARN << "Duplicate smoother plugin id ignored: " << spec.id;
            continue;
        }
        if (!loader.IsSmootherRegistered(spec.type)) {
            AFATAL << "Unknown smoother plugin type: " << spec.type;
            return;
        }

        auto smoother = loader.CreateSmoother(spec.type);
        if (!smoother) {
            AFATAL << "Failed to create smoother: " << spec.id;
            return;
        }

        ApplySmootherOptions(*smoother, options_, spec.id);
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
            ApplySmootherOptions(*entry.second, options_, entry.first);
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

    std::string resolved_id = smoother_id.empty() ? default_smoother_id_
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

}  // namespace planning
}  // namespace autonomy

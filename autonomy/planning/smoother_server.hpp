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

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include <unordered_map>

#include "autolink/autolink.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/planning/common/smoother_interface.hpp"
#include "autonomy/planning/proto/planning_options.pb.h"

namespace autonomy {
namespace planning {

/**
 * @brief Result of a single SmoothPath invocation.
 */
struct SmoothPathResult {
    /** Smoothed path (poses may be updated in place by the plugin). */
    commsgs::planning_msgs::Path path;
    /** True if the smoother finished within max_time. */
    bool was_completed{false};
    /** Wall-clock smoothing duration in seconds. */
    double smoothing_duration_sec{0.0};
};

/**
 * Smoother server contract:
 * - Plugins are loaded from PlannerOptions.smoother_plugins via PluginManager.
 * - Empty smoother_id selects default_smoother_id (from config or
 *   simple_smoother).
 * - SmoothPath throws common::SmootherException subclasses on invalid input,
 *   unknown smoother, timeout, or collision when check_for_collisions is true.
 * - Optional costmap_wrapper is used only for post-smooth collision checks.
 */
class SmootherServer
{
public:
    /**
     * @brief Map of smoother instance id to plugin implementation.
     */
    using SmootherMap =
        std::unordered_map<std::string, common::Smoother::SharedPtr>;

    /**
     * @brief Define SmootherServer::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(SmootherServer)

    /**
     * @brief Construct the smoother server and load configured plugins.
     * @param options Planner options (smoother list and per-plugin settings).
     * @param costmap_wrapper Costmap for collision checks; may be nullptr.
     */
    SmootherServer(const proto::PlannerOptions& options,
                   map::costmap_2d::Costmap2DWrapper::SharedPtr costmap_wrapper);

    /**
     * @brief Destructor; calls Shutdown() if not already shut down.
     */
    ~SmootherServer();

    /**
     * @brief Activate all loaded smoother plugins.
     */
    void Start();

    /**
     * @brief Deactivate and clean up all smoother plugins.
     */
    void Shutdown();

    /**
     * @brief Default smoother id used when SmoothPath receives an empty id.
     * @return Configured default (e.g. simple_smoother).
     */
    const std::string& GetDefaultSmootherId() const {
        return default_smoother_id_;
    }

    using PathUpdateCallback =
        std::function<void(const commsgs::planning_msgs::Path&)>;
    void SetPathUpdateCallback(PathUpdateCallback callback);

    /**
     * @brief Smooth a path with the selected plugin.
     *
     * @param path Input path (at least two poses); modified in place on success.
     * @param smoother_id Plugin instance id; empty uses default_smoother_id_.
     * @param max_time Maximum time allowed for the smoothing algorithm.
     * @param check_for_collisions If true, verify smoothed path against
     * costmap_wrapper_.
     * @param cancel_checker Optional predicate; true throws SmootherTimedOut.
     * @return SmoothPathResult with path, completion flag, and duration.
     * @throws common::InvalidPath Path has fewer than two poses.
     * @throws common::InvalidSmoother Unknown smoother_id.
     * @throws common::SmootherTimedOut Cancelled or exceeded max_time.
     * @throws common::SmoothedPathInCollision Collision check failed.
     */
    SmoothPathResult SmoothPath(
        commsgs::planning_msgs::Path path, const std::string& smoother_id,
        const std::chrono::milliseconds& max_time, bool check_for_collisions,
        const std::function<bool()>& cancel_checker);

    bool AttachAutolinkNode(std::shared_ptr<autolink::Node> node);
    void DetachAutolinkNode();

private:
    /**
     * @brief Load smoother plugins from options via PluginManager.
     */
    void LoadPlugins();

    /**
     * @brief Return true if any pose projects to lethal or unknown cost.
     * @param path Path to check in costmap_wrapper_ global frame.
     * @return true if in collision or costmap unavailable.
     */
    bool IsPathInCollision(const commsgs::planning_msgs::Path& path) const;

    // Planner / smoother configuration
    proto::PlannerOptions options_;

    // Costmap used for optional post-smooth collision validation
    map::costmap_2d::Costmap2DWrapper::SharedPtr costmap_wrapper_;

    // Loaded smoother plugins keyed by config id
    SmootherMap smoothers_;
    /** Space-separated list of loaded ids (for error messages). */
    std::string smoother_ids_concat_;
    std::string default_smoother_id_;
    PathUpdateCallback path_update_callback_;
    bool plugins_loaded_{false};
    std::atomic<bool> shutdown_called_{false};

    struct AutolinkActionServers;
    AutolinkActionServers* autolink_actions_{nullptr};
};

}  // namespace planning
}  // namespace autonomy

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

struct SmoothPathResult {
    commsgs::planning_msgs::Path path;
    bool was_completed{false};
    double smoothing_duration_sec{0.0};
};

class SmootherServer
{
public:
    using SmootherMap =
        std::unordered_map<std::string, common::Smoother::SharedPtr>;

    AUTONOMY_SMART_PTR_DEFINITIONS(SmootherServer)

    SmootherServer(const proto::PlannerOptions& options,
                   map::costmap_2d::Costmap2DWrapper::SharedPtr costmap_wrapper);

    ~SmootherServer();

    void Start();
    void Shutdown();

    const std::string& GetDefaultSmootherId() const {
        return default_smoother_id_;
    }

    using PathUpdateCallback =
        std::function<void(const commsgs::planning_msgs::Path&)>;
    void SetPathUpdateCallback(PathUpdateCallback callback);

    SmoothPathResult SmoothPath(
        commsgs::planning_msgs::Path path, const std::string& smoother_id,
        const std::chrono::milliseconds& max_time, bool check_for_collisions,
        const std::function<bool()>& cancel_checker);

    bool AttachAutolinkNode(std::shared_ptr<autolink::Node> node);
    void DetachAutolinkNode();

private:
    void LoadPlugins();
    bool IsPathInCollision(const commsgs::planning_msgs::Path& path) const;

    proto::PlannerOptions options_;
    SmootherMap smoothers_;
    std::string smoother_ids_concat_;
    std::string default_smoother_id_;
    bool plugins_loaded_{false};
    map::costmap_2d::Costmap2DWrapper::SharedPtr costmap_wrapper_;
    std::atomic<bool> shutdown_called_{false};
    PathUpdateCallback path_update_callback_;

    struct AutolinkActionServers;
    AutolinkActionServers* autolink_actions_{nullptr};
};

}  // namespace planning
}  // namespace autonomy

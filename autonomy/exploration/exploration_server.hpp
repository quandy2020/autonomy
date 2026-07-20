/*
 * Copyright 2026 The Openbot Authors
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

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <chrono>

#include "autolink/autolink.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/exploration/common/explorer_interface.hpp"
#include "autonomy/exploration/proto/exploration_options.pb.h"

namespace autonomy {
namespace exploration {

/**
 * @file exploration_server.hpp
 * @brief Exploration process server (mirrors control::ControllerServer).
 */

/**
 * @class ExplorationServer
 * @brief Owns explorer plugins and exposes online update / waypoint APIs.
 */
class ExplorationServer
{
public:
    /**
     * @brief Map from explorer plugin id to instance.
     */
    using ExplorerMap =
        std::unordered_map<std::string, common::ExplorerInterface::SharedPtr>;

    AUTONOMY_SMART_PTR_DEFINITIONS(ExplorationServer)

    /**
     * @brief Constructor with exploration options.
     * @param options Exploration options
     */
    explicit ExplorationServer(const proto::ExplorationOptions& options);

    /**
     * @brief Destructor; shuts down if still running.
     */
    ~ExplorationServer();

    /**
     * @brief Start the Autolink node and attach servers.
     */
    void Start();

    /**
     * @brief Detach Autolink and stop the server.
     */
    void Shutdown();

    /**
     * @brief Attach an existing Autolink node.
     * @param node Autolink node
     * @return true on success
     */
    bool AttachAutolinkNode(std::shared_ptr<autolink::Node> node);

    /**
     * @brief Detach Autolink action servers / node bindings.
     */
    void DetachAutolinkNode();

    /**
     * @brief Forward odometry to the active explorer.
     * @param odom Robot odometry
     */
    void UpdateOdometry(const commsgs::planning_msgs::Odometry& odom);

    /**
     * @brief Forward depth to the active explorer and replan.
     * @param depth Depth image
     * @param info Camera intrinsics
     * @param map_t_camera Extrinsic transform from camera to map
     */
    void UpdateDepth(const commsgs::sensor_msgs::Image& depth,
                     const commsgs::sensor_msgs::CameraInfo& info,
                     const commsgs::geometry_msgs::Transform& map_t_camera);

    /**
     * @brief Set exploration area on the active explorer.
     * @param area Boundary polygon
     */
    void SetExplorationArea(const commsgs::geometry_msgs::Polygon& area);

    /**
     * @brief Get the next exploration waypoint.
     * @param out Output pose
     * @return true if a waypoint is available
     */
    bool GetNextWaypoint(commsgs::geometry_msgs::PoseStamped& out);

    /**
     * @brief Mark the current waypoint as reached.
     */
    void MarkWaypointReached();

    /**
     * @brief Check whether the active explorer has a target.
     * @return true if a target exists
     */
    bool HasExplorationTarget() const;

    /**
     * @brief Check whether the active explorer finished.
     * @return true if finished
     */
    bool IsFinished() const;

    /**
     * @brief Get progress from the active explorer.
     * @return Progress in [0, 1]
     */
    float Progress() const;

    /**
     * @brief Get explored area from the active explorer.
     * @return Area [m^2]
     */
    float ExploredAreaM2() const;

    /**
     * @brief Resolve an explorer plugin id.
     * @param requested Requested id (empty => active)
     * @param name Output resolved id
     * @return true if found
     */
    bool FindExplorerId(const std::string& requested, std::string* name) const;

    /**
     * @brief Get the currently active explorer instance.
     * @return Shared pointer, or nullptr
     */
    common::ExplorerInterface::SharedPtr ActiveExplorer() const;

protected:
    /**
     * @brief Instantiate explorer plugins from options.
     */
    void LoadExplorers();

    /**
     * @brief Trigger ExecutePlanningCycle on the active explorer.
     */
    void ComputeExploration();

    proto::ExplorationOptions options_;  //!< @brief exploration options
    ExplorerMap explorers_;              //!< @brief loaded explorer plugins
    std::string active_explorer_id_;     //!< @brief active plugin id
    std::shared_ptr<autolink::Node> node_;  //!< @brief Autolink node
    mutable std::mutex mutex_;           //!< @brief guards explorer access
    bool started_{false};                //!< @brief Start() has been called
    std::chrono::steady_clock::time_point
        last_plan_time_{};               //!< @brief last planning cycle time
    bool has_last_plan_time_{false};     //!< @brief whether last_plan_time_ set

    /**
     * @brief Reserved for future GetWaypoint Autolink action servers.
     */
    struct AutolinkActionServers {};
    std::unique_ptr<AutolinkActionServers>
        autolink_actions_;  //!< @brief Autolink action servers holder
};

}  // namespace exploration
}  // namespace autonomy

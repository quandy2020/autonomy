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
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/system/proto/autonomy_options.pb.h"
#include "autonomy/navigator/constants.hpp"
#include "autonomy/navigator/options.hpp"
#include "autonomy/sensor/collator_interface.hpp"

namespace autonomy {
namespace control {
class ControllerServer;
}
namespace map {
class MapServer;
}
namespace planning {
class PlannerServer;
}
namespace sensor {
class SensorCollator;
}
namespace transform {
class Buffer;
class TransformServer;
}
namespace system {

/** Runtime overrides from ROS parameters or Autonomy::Configure(). */
struct RuntimeOptions {
    bool enable_bt_tasks{true};
    bool use_bt_navigation{true};
    std::string config_directory;
    std::string planner_id;
    std::string controller_id;
    std::string goal_checker_id;
    std::string progress_checker_id;
    std::string global_frame;
    std::string robot_base_frame;
    double goal_tolerance{0.15};
};

/** Top-level autonomy process: map / planner / controller / navigator. */
class Autonomy
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(Autonomy)

    using MapPublishListener = std::function<void(
        const commsgs::map_msgs::OccupancyGrid::SharedPtr& map)>;
    using PathListener =
        std::function<void(const commsgs::planning_msgs::Path& path)>;

    explicit Autonomy(proto::AutonomyOptions options);
    ~Autonomy();

    Autonomy(const Autonomy&) = delete;
    Autonomy& operator=(const Autonomy&) = delete;

    void Start();
    void Configure(const RuntimeOptions& runtime = {});
    void Shutdown();

    bool IsReady() const;
    bool UseBehaviorTreeNavigation() const { return use_bt_navigation_; }

    map::MapServer* GetMapServer();
    map::costmap_2d::Costmap2DWrapper::SharedPtr GetGlobalCostmap();
    sensor::CollatorInterface& GetSensorCollator();
    control::ControllerServer* GetController();

    void AddMapPublishListener(MapPublishListener listener);
    void AddPathListener(PathListener listener);

    bool NavigateToPose(
        const commsgs::geometry_msgs::PoseStamped& goal,
        std::function<bool()> cancel_checker,
        std::function<bool()> keep_alive,
        double timeout_sec = navigator::kDirectNavDefaultTimeoutSec);

    bool NavigateThroughPoses(
        const std::vector<commsgs::geometry_msgs::PoseStamped>& goals,
        std::function<bool()> cancel_checker,
        std::function<bool()> keep_alive,
        double timeout_sec = navigator::kDirectNavDefaultTimeoutSec);

    void ReplanToGoal(const commsgs::geometry_msgs::PoseStamped& goal);
    std::optional<commsgs::planning_msgs::Path> GetLastPath();

    void RequestCancelNavigation();
    void SetControllerEnabled(bool enabled);

    commsgs::geometry_msgs::TwistStamped TickControl();

    /** Latest velocity from direct follow or BT (for in-process simulators). */
    commsgs::geometry_msgs::TwistStamped GetLastControlCommand() const;

    bool TransformPoseToGlobalFrame(
        commsgs::geometry_msgs::PoseStamped& pose);

private:
    struct PluginIds {
        std::string planner_id;
        std::string controller_id;
        std::string goal_checker_id;
    };

    bool EnsureStarted() const;
    PluginIds ResolvePluginIds() const;
    bool NavigateDirectToPose(
        const commsgs::geometry_msgs::PoseStamped& goal,
        std::function<bool()> cancel_checker, std::function<bool()> keep_alive,
        double timeout_sec);
    bool GetRobotPose(commsgs::geometry_msgs::PoseStamped& pose) const;
    void ApplyMapToCostmap(
        const commsgs::map_msgs::OccupancyGrid::SharedPtr& map);
    void NotifyPath(const commsgs::planning_msgs::Path& path);
    void ApplyRuntimeToNavigatorOptions(const RuntimeOptions& runtime);
    void SyncNavigationFrames(const std::string& global_frame,
                              const std::string& robot_base_frame);

    proto::AutonomyOptions options_;
    RuntimeOptions runtime_;
    navigator::proto::NavigatorOptions navigator_options_;

    std::shared_ptr<map::MapServer> map_server_;
    std::shared_ptr<planning::PlannerServer> planner_;
    std::shared_ptr<control::ControllerServer> controller_;
    std::shared_ptr<transform::Buffer> tf_buffer_;
    std::unique_ptr<transform::TransformServer> transform_server_;
    std::unique_ptr<sensor::SensorCollator> sensor_collator_;

    std::vector<MapPublishListener> map_listeners_;
    std::vector<PathListener> path_listeners_;
    std::optional<commsgs::planning_msgs::Path> last_path_;

    mutable std::mutex mutex_;
    std::atomic<bool> started_{false};
    std::atomic<bool> configured_{false};
    std::atomic<bool> controller_enabled_{true};
    std::atomic<bool> use_bt_navigation_{false};
    commsgs::geometry_msgs::TwistStamped last_control_cmd_;
};

Autonomy::UniquePtr CreateAutonomy(
    const proto::AutonomyOptions& options);

}  // namespace system
}  // namespace autonomy

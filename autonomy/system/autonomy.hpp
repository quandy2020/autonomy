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
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/map_msgs/map_msgs.pb.h>
#include <automsgs/msgs/nav_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/planning_msgs/planning_msgs.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
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
        const std::shared_ptr<automsgs::msgs::map_msgs::OccupancyGrid>& map)>;
    using PathListener =
        std::function<void(const automsgs::msgs::planning_msgs::Path& path)>;

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
        const automsgs::msgs::geometry_msgs::PoseStamped& goal,
        std::function<bool()> cancel_checker,
        std::function<bool()> keep_alive,
        double timeout_sec = navigator::kDirectNavDefaultTimeoutSec);

    bool NavigateThroughPoses(
        const std::vector<automsgs::msgs::geometry_msgs::PoseStamped>& goals,
        std::function<bool()> cancel_checker,
        std::function<bool()> keep_alive,
        double timeout_sec = navigator::kDirectNavDefaultTimeoutSec);

    void ReplanToGoal(const automsgs::msgs::geometry_msgs::PoseStamped& goal);
    std::optional<automsgs::msgs::planning_msgs::Path> GetLastPath();

    void RequestCancelNavigation();
    void SetControllerEnabled(bool enabled);

    automsgs::msgs::geometry_msgs::TwistStamped TickControl();

    /** Latest velocity from direct follow or BT (for in-process simulators). */
    automsgs::msgs::geometry_msgs::TwistStamped GetLastControlCommand() const;

    bool TransformPoseToGlobalFrame(
        automsgs::msgs::geometry_msgs::PoseStamped& pose);

private:
    struct PluginIds {
        std::string planner_id;
        std::string controller_id;
        std::string goal_checker_id;
    };

    bool EnsureStarted() const;
    PluginIds ResolvePluginIds() const;
    bool NavigateDirectToPose(
        const automsgs::msgs::geometry_msgs::PoseStamped& goal,
        std::function<bool()> cancel_checker, std::function<bool()> keep_alive,
        double timeout_sec);
    bool GetRobotPose(automsgs::msgs::geometry_msgs::PoseStamped& pose) const;
    void ApplyMapToCostmap(
        const std::shared_ptr<automsgs::msgs::map_msgs::OccupancyGrid>& map);
    void NotifyPath(const automsgs::msgs::planning_msgs::Path& path);
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
    std::optional<automsgs::msgs::planning_msgs::Path> last_path_;

    mutable std::mutex mutex_;
    std::atomic<bool> started_{false};
    std::atomic<bool> configured_{false};
    std::atomic<bool> controller_enabled_{true};
    std::atomic<bool> use_bt_navigation_{false};
    automsgs::msgs::geometry_msgs::TwistStamped last_control_cmd_;
};

Autonomy::UniquePtr CreateAutonomy(
    const proto::AutonomyOptions& options);

}  // namespace system
}  // namespace autonomy

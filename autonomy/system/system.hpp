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

#include <functional>
#include <memory>
#include <optional>

#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/sensor/collator_interface.hpp"
#include "autonomy/system/proto/autonomy_options.pb.h"
#include "autonomy/tasks/task.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {
class Costmap2DWrapper;
}  // namespace costmap_2d
}  // namespace map
namespace system {

using RuntimeOptions = tasks::RuntimeOptions;

/**
 * @brief Core autonomy facade: all navigation, sensing ingress, and control live here.
 *
 * ROS and other frontends only convert I/O and call these APIs.
 */
class AutonomyNode
{
public:
  AUTONOMY_SMART_PTR_DEFINITIONS(AutonomyNode)

  AutonomyNode() = default;

  explicit AutonomyNode(const proto::AutonomyOptions & options);

  ~AutonomyNode();

  void Start();

  void Shutdown();

  /** @brief Planner / controller IDs, BT attach, navigation parameters. */
  void configure(const RuntimeOptions & options);

  tasks::Task & task() { return *task_; }

  const tasks::Task & task() const { return *task_; }

  map::MapServer * map_server() { return task_->map_server(); }

  planning::PlannerServer * planner_server() { return task_->planner_server(); }

  planning::SmootherServer * smoother_server() { return task_->smoother_server(); }

  tasks::common::TaskContext * task_context() { return task_->task_context(); }

  control::ControllerServer * controller_server() {
    return task_->controller_server();
  }

  map::costmap_2d::Costmap2DWrapper * globalCostmap();

  using MapPublishListener = std::function<void(
    const commsgs::map_msgs::OccupancyGrid::SharedPtr & map)>;
  using PathListener = std::function<void(const commsgs::planning_msgs::Path & path)>;

  void addMapPublishListener(MapPublishListener listener);
  void addPathListener(PathListener listener);

  sensor::CollatorInterface & sensorCollator();
  const sensor::CollatorInterface & sensorCollator() const;

  bool planToGoal(const commsgs::geometry_msgs::PoseStamped & goal);
  void replanToGoal(const commsgs::geometry_msgs::PoseStamped & goal);

  bool navigateToPose(
    const commsgs::geometry_msgs::PoseStamped & goal,
    std::function<bool()> cancel_checker = nullptr,
    std::function<bool()> continue_predicate = nullptr,
    double timeout_sec = 0.0);

  void setControllerEnabled(bool enabled);
  void requestCancelNavigation();

  /** @brief One control period: BT relay or TickFollowPath; returns latest cmd_vel. */
  commsgs::geometry_msgs::TwistStamped tickControl();

  std::optional<commsgs::planning_msgs::Path> lastPath() const;

  bool transformPoseToGlobalFrame(commsgs::geometry_msgs::PoseStamped & pose);

  bool isSchedulerReady() const { return task_->isSchedulerReady(); }

  bool useBehaviorTreeNavigation() const;

  /** @brief 多点途经导航（BT）。 */
  tasks::behavior_tree::BtStatus navigateThroughPoses(
    const std::vector<commsgs::geometry_msgs::PoseStamped> & poses,
    const std::string & behavior_tree = "");

  /** @brief 自动回充：导航至 dock 位姿（BT）。 */
  tasks::behavior_tree::BtStatus navigateToDock(
    const commsgs::geometry_msgs::PoseStamped & dock_pose,
    const std::string & dock_type = "default",
    const std::string & dock_id = "");

  /** @brief 人体 / 目标跟随（BT）。 */
  tasks::behavior_tree::BtStatus trackToTarget(uint32_t target_id);

  /** @brief 环境探索（BT）。 */
  tasks::behavior_tree::BtStatus exploreAnywhere(double time_allowance_sec = 0.0);

  tasks::behavior_tree::BtStatus teleopDrive(
    double time_allowance_sec = 0.0,
    double max_linear_vel = 0.0,
    double max_angular_vel = 0.0,
    std::function<bool()> cancel_checker = nullptr);

  void beginTeleop(double max_linear_vel = 0.0, double max_angular_vel = 0.0);
  void endTeleop();
  bool isTeleopActive() const;
  bool updateTeleopCommand(const commsgs::geometry_msgs::TwistStamped & cmd);

  bool updateTrackTargetPose(const commsgs::geometry_msgs::PoseStamped & pose);
  bool updateExploreGoal(const commsgs::geometry_msgs::PoseStamped & goal);

  bool hasNavigator(const std::string & id) const;
  std::vector<std::string> registeredNavigatorIds() const;

private:
  std::unique_ptr<tasks::Task> task_;
};

AutonomyNode::UniquePtr CreateAutonomy(const proto::AutonomyOptions & options);

}  // namespace system
}  // namespace autonomy

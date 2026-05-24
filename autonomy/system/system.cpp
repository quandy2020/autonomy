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

#include "autonomy/system/system.hpp"

#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy {
namespace system {

AutonomyNode::AutonomyNode(const proto::AutonomyOptions & options)
: task_(std::make_unique<tasks::Task>(options))
{
}

AutonomyNode::~AutonomyNode() = default;

void AutonomyNode::Start()
{
  if (task_) {
    task_->start();
  }
}

void AutonomyNode::Shutdown()
{
  if (task_) {
    task_->shutdown();
  }
}

void AutonomyNode::configure(const RuntimeOptions & options)
{
  if (task_) {
    task_->configure(options);
  }
}

map::costmap_2d::Costmap2DWrapper * AutonomyNode::globalCostmap()
{
  if (!task_ || !task_->planner_server()) {
    return nullptr;
  }
  return task_->planner_server()->GetCostmapWrapper().get();
}

void AutonomyNode::addMapPublishListener(MapPublishListener listener)
{
  if (task_) {
    task_->addMapPublishListener(std::move(listener));
  }
}

void AutonomyNode::addPathListener(PathListener listener)
{
  if (task_) {
    task_->addPathListener(std::move(listener));
  }
}

void AutonomyNode::updateOdometry(const commsgs::planning_msgs::Odometry & odom)
{
  if (task_) {
    task_->updateOdometry(odom);
  }
}

void AutonomyNode::feedLaserScan(const commsgs::sensor_msgs::LaserScan & scan)
{
  if (task_) {
    task_->feedLaserScan(scan);
  }
}

void AutonomyNode::feedPointCloud2(const commsgs::sensor_msgs::PointCloud2 & cloud)
{
  if (task_) {
    task_->feedPointCloud2(cloud);
  }
}

void AutonomyNode::feedRange(const commsgs::sensor_msgs::Range & range)
{
  if (task_) {
    task_->feedRange(range);
  }
}

bool AutonomyNode::planToGoal(const commsgs::geometry_msgs::PoseStamped & goal)
{
  return task_ && task_->planToGoal(goal);
}

void AutonomyNode::replanToGoal(const commsgs::geometry_msgs::PoseStamped & goal)
{
  if (task_) {
    task_->replanToGoal(goal);
  }
}

bool AutonomyNode::navigateToPose(
  const commsgs::geometry_msgs::PoseStamped & goal,
  std::function<bool()> cancel_checker,
  std::function<bool()> continue_predicate,
  const double timeout_sec)
{
  return task_ && task_->navigateToPose(
    goal, std::move(cancel_checker), std::move(continue_predicate), timeout_sec);
}

void AutonomyNode::setControllerEnabled(bool enabled)
{
  if (task_) {
    task_->setControllerEnabled(enabled);
  }
}

void AutonomyNode::requestCancelNavigation()
{
  if (task_) {
    task_->requestCancelNavigation();
  }
}

void AutonomyNode::applySpeedLimit(const commsgs::planning_msgs::SpeedLimit & limit)
{
  if (task_) {
    task_->applySpeedLimit(limit);
  }
}

void AutonomyNode::clearSpeedLimit()
{
  if (task_) {
    task_->clearSpeedLimit();
  }
}

commsgs::geometry_msgs::TwistStamped AutonomyNode::tickControl()
{
  if (!task_) {
    return {};
  }
  return task_->tickControl();
}

std::optional<commsgs::planning_msgs::Path> AutonomyNode::lastPath() const
{
  if (!task_) {
    return std::nullopt;
  }
  return task_->lastPath();
}

bool AutonomyNode::transformPoseToGlobalFrame(
  commsgs::geometry_msgs::PoseStamped & pose)
{
  return task_ && task_->transformPoseToGlobalFrame(pose);
}

bool AutonomyNode::useBehaviorTreeNavigation() const
{
  return task_ && task_->useBehaviorTreeNavigation();
}

tasks::behavior_tree::BtStatus AutonomyNode::navigateThroughPoses(
  const std::vector<commsgs::geometry_msgs::PoseStamped> & poses,
  const std::string & behavior_tree)
{
  if (!task_) {
    return tasks::behavior_tree::BtStatus::FAILED;
  }
  return task_->navigateThroughPoses(poses, behavior_tree);
}

tasks::behavior_tree::BtStatus AutonomyNode::navigateToDock(
  const commsgs::geometry_msgs::PoseStamped & dock_pose,
  const std::string & dock_type, const std::string & dock_id)
{
  if (!task_) {
    return tasks::behavior_tree::BtStatus::FAILED;
  }
  return task_->navigateToDock(dock_pose, dock_type, dock_id);
}

tasks::behavior_tree::BtStatus AutonomyNode::trackToTarget(const uint32_t target_id)
{
  if (!task_) {
    return tasks::behavior_tree::BtStatus::FAILED;
  }
  return task_->trackToTarget(target_id);
}

tasks::behavior_tree::BtStatus AutonomyNode::exploreAnywhere(
  const double time_allowance_sec)
{
  if (!task_) {
    return tasks::behavior_tree::BtStatus::FAILED;
  }
  return task_->exploreAnywhere(time_allowance_sec);
}

bool AutonomyNode::updateTrackTargetPose(
  const commsgs::geometry_msgs::PoseStamped & pose)
{
  return task_ && task_->updateTrackTargetPose(pose);
}

bool AutonomyNode::updateExploreGoal(
  const commsgs::geometry_msgs::PoseStamped & goal)
{
  return task_ && task_->updateExploreGoal(goal);
}

bool AutonomyNode::hasNavigator(const std::string & id) const
{
  return task_ && task_->hasNavigator(id);
}

std::vector<std::string> AutonomyNode::registeredNavigatorIds() const
{
  if (!task_) {
    return {};
  }
  return task_->registeredNavigatorIds();
}

tasks::behavior_tree::BtStatus AutonomyNode::teleopDrive(
  const double time_allowance_sec,
  const double max_linear_vel,
  const double max_angular_vel,
  std::function<bool()> cancel_checker)
{
  if (!task_) {
    return tasks::behavior_tree::BtStatus::FAILED;
  }
  return task_->teleopDrive(
    time_allowance_sec, max_linear_vel, max_angular_vel, std::move(cancel_checker));
}

void AutonomyNode::beginTeleop(
  const double max_linear_vel, const double max_angular_vel)
{
  if (task_) {
    task_->beginTeleop(max_linear_vel, max_angular_vel);
  }
}

void AutonomyNode::endTeleop()
{
  if (task_) {
    task_->endTeleop();
  }
}

bool AutonomyNode::isTeleopActive() const
{
  return task_ && task_->isTeleopActive();
}

bool AutonomyNode::updateTeleopCommand(
  const commsgs::geometry_msgs::TwistStamped & cmd)
{
  return task_ && task_->updateTeleopCommand(cmd);
}

AutonomyNode::UniquePtr CreateAutonomy(const proto::AutonomyOptions & options)
{
  return std::make_unique<AutonomyNode>(options);
}

}  // namespace system
}  // namespace autonomy

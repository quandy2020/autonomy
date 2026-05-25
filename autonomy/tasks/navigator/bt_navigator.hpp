/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/behavior_tree/bt_engine.hpp"
#include "autonomy/tasks/proto/bt_action.pb.h"

namespace autonomy {
namespace control {
class ControllerServer;
}
namespace tasks {
namespace proto {
class TaskOptions;
}
namespace common {
class TaskContext;
}
namespace navigator {
namespace teleop {
class TeleopSession;
}

/**
 * @brief Registers and runs in-process BT navigators (one instance per tasks.lua id).
 *
 * Not to be confused with common::BtNavigator<ActionT>, which executes a single
 * behavior tree for one action type.
 */
class NavigatorRegistry
{
public:
  AUTONOMY_SMART_PTR_DEFINITIONS(NavigatorRegistry)

  NavigatorRegistry();
  ~NavigatorRegistry();

  void attach(
    const std::string & config_directory,
    std::shared_ptr<common::TaskContext> task_context,
    std::shared_ptr<control::ControllerServer> controller);
  void shutdown();
  bool isReady() const;

  void requestCancel();

  behavior_tree::BtStatus navigateToPose(
    std::shared_ptr<const proto::NavigateToPoseAction::Goal> goal);
  behavior_tree::BtStatus navigateThroughPoses(
    const std::vector<commsgs::geometry_msgs::PoseStamped> & poses,
    const std::string & behavior_tree = "");
  behavior_tree::BtStatus navigateToDock(
    const commsgs::geometry_msgs::PoseStamped & dock_pose,
    const std::string & dock_type = "default",
    const std::string & dock_id = "");
  behavior_tree::BtStatus trackToTarget(uint32_t target_id);
  behavior_tree::BtStatus exploreAnywhere(double time_allowance_sec = 0.0);
  behavior_tree::BtStatus teleopDrive(
    double time_allowance_sec = 0.0,
    double max_linear_vel = 0.0,
    double max_angular_vel = 0.0,
    std::function<bool()> cancel_checker = nullptr);

  /** Active teleop_drive navigator session; null if teleop is disabled. */
  teleop::TeleopSession * teleopSession();
  const teleop::TeleopSession * teleopSession() const;

  bool updateTrackTargetPose(const commsgs::geometry_msgs::PoseStamped & pose);
  bool updateExploreGoal(const commsgs::geometry_msgs::PoseStamped & goal);

  bool hasNavigator(const std::string & id) const;
  std::vector<std::string> registeredNavigatorIds() const;

  const proto::TaskOptions & taskOptions() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy

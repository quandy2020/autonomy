/*
 * Copyright 2026 autonomy contributors
 */

#pragma once

#include <string>

#include "autonomy/tasks/behavior_tree/bt_stateful_action_node.hpp"
#include "autonomy/tasks/navigator/teleop/teleop_session.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/**
 * @brief In-process teleop session BT node (runs @ref navigator::teleop::TeleopSession).
 */
class TeleopDriveAction : public BtStatefulActionNode
{
public:
  TeleopDriveAction(
    const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("time_allowance", 0.0, "Session duration (s); 0 = unlimited"),
      BT::InputPort<double>("max_linear_vel", 0.0, "Linear speed limit (m/s); 0 = use config"),
      BT::InputPort<double>("max_angular_vel", 0.0, "Angular speed limit (rad/s); 0 = use config"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<navigator::teleop::TeleopSession> session_;
  double time_allowance_sec_{0.0};
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

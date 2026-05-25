/*
 * Copyright 2026 autonomy contributors
 */

#pragma once

#include <string>

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "autonomy/tasks/navigator/teleop/teleop_session.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/**
 * @brief Assisted continuous teleop: filters streamed cmd_vel against costmap.
 */
class AssistedTeleopVelocityAction : public BtStatefulActionNode
{
public:
  AssistedTeleopVelocityAction(
    const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("time_allowance", 0.0, "Session duration (s); 0 = unlimited"),
      BT::InputPort<double>("max_linear_vel", 0.0, "Linear speed cap (m/s); 0 = use config"),
      BT::InputPort<double>("max_angular_vel", 0.0, "Angular speed cap (rad/s); 0 = use config"),
      BT::InputPort<bool>("disable_collision_checks", false,
                          "Skip costmap collision filtering"),
      BT::InputPort<double>("projection_time_sec", 1.5, "Forward simulation horizon (s)"),
      BT::InputPort<double>("simulation_step_sec", 0.1, "Simulation dt (s)"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<navigator::teleop::TeleopSession> session_;
  double time_allowance_sec_{0.0};
  bool disable_collision_checks_{false};
  double projection_time_sec_{1.5};
  double simulation_step_sec_{0.1};
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

/*
 * Copyright 2026 autonomy contributors
 */

#include "autonomy/tasks/behavior_tree/plugins/action/teleop_drive_action.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/navigator/teleop/teleop_session.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

TeleopDriveAction::TeleopDriveAction(
  const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BtStatefulActionNode(xml_tag_name, conf)
{
}

BT::NodeStatus TeleopDriveAction::onStart()
{
  if (!config().blackboard) {
    AERROR << "TeleopDrive: blackboard missing";
    return BT::NodeStatus::FAILURE;
  }
  session_ =
    config().blackboard->get<std::shared_ptr<navigator::teleop::TeleopSession>>(
      "teleop_session");
  if (!session_) {
    AERROR << "TeleopDrive: teleop_session missing on blackboard";
    return BT::NodeStatus::FAILURE;
  }

  time_allowance_sec_ = 0.0;
  getInput("time_allowance", time_allowance_sec_);

  double max_linear = 0.0;
  double max_angular = 0.0;
  getInput("max_linear_vel", max_linear);
  getInput("max_angular_vel", max_angular);

  if (config().blackboard) {
    config().blackboard->get("teleop_max_linear_vel", max_linear);   // NOLINT
    config().blackboard->get("teleop_max_angular_vel", max_angular);  // NOLINT
    if (time_allowance_sec_ <= 0.0) {
      config().blackboard->get("teleop_time_allowance", time_allowance_sec_);  // NOLINT
    }
  }

  navigator::teleop::TeleopSession::Limits limits;
  if (config().blackboard) {
    config().blackboard->get("teleop_cmd_stale_timeout_sec",  // NOLINT
                            limits.cmd_stale_timeout_sec);
  }
  if (max_linear > 0.0) {
    limits.max_linear_vel = max_linear;
  }
  if (max_angular > 0.0) {
    limits.max_angular_vel = max_angular;
  }

  session_->Begin(time_allowance_sec_, limits);
  AINFO << "[TeleopDrive] BT session started (allowance=" << time_allowance_sec_
        << "s)";
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TeleopDriveAction::onRunning()
{
  if (!session_ || !session_->IsActive()) {
    return BT::NodeStatus::SUCCESS;
  }
  if (isCancelRequested()) {
    return BT::NodeStatus::FAILURE;
  }
  if (!session_->Tick([this]() { return isCancelRequested(); })) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void TeleopDriveAction::onHalted()
{
  if (session_) {
    session_->End();
  }
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<
    autonomy::tasks::behavior_tree::plugins::action::TeleopDriveAction>(
    "TeleopDrive");
}

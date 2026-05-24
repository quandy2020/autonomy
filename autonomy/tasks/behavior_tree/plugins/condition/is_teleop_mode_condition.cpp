/*
 * Copyright 2026 autonomy contributors
 */

#include "autonomy/tasks/behavior_tree/plugins/condition/is_teleop_mode_condition.hpp"

#include "autonomy/tasks/navigator/proto/action.pb.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

namespace {

using Mode = proto::TeleopMotionMode;

Mode ModeFromString(const std::string & name)
{
  if (name == "velocity" || name == "manual") {
    return Mode::TELEOP_MOTION_VELOCITY;
  }
  if (name == "raw") {
    return Mode::TELEOP_MOTION_RAW;
  }
  if (name == "forward") {
    return Mode::TELEOP_MOTION_FORWARD;
  }
  if (name == "backward") {
    return Mode::TELEOP_MOTION_BACKWARD;
  }
  if (name == "rotate") {
    return Mode::TELEOP_MOTION_ROTATE;
  }
  if (name == "sequence") {
    return Mode::TELEOP_MOTION_SEQUENCE;
  }
  return Mode::TELEOP_MOTION_VELOCITY;
}

}  // namespace

IsTeleopMode::IsTeleopMode(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus IsTeleopMode::tick()
{
  std::string expected = "velocity";
  getInput("mode", expected);

  int mode_value = static_cast<int>(Mode::TELEOP_MOTION_VELOCITY);
  if (config().blackboard) {
    config().blackboard->get("teleop_mode", mode_value);  // NOLINT
  }

  const auto expected_mode = static_cast<int>(ModeFromString(expected));
  return mode_value == expected_mode ? BT::NodeStatus::SUCCESS
                                     : BT::NodeStatus::FAILURE;
}

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<
    autonomy::tasks::behavior_tree::plugins::condition::IsTeleopMode>(
    "IsTeleopMode");
}

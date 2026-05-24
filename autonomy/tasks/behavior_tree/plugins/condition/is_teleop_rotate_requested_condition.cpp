/*
 * Copyright 2026 autonomy contributors
 */

#include "autonomy/tasks/behavior_tree/plugins/condition/is_teleop_rotate_requested_condition.hpp"

#include <cmath>

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

IsTeleopRotateRequested::IsTeleopRotateRequested(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus IsTeleopRotateRequested::tick()
{
  double rotation_angle = 0.0;
  if (config().blackboard) {
    config().blackboard->get("teleop_rotation_angle", rotation_angle);  // NOLINT
  }
  return std::abs(rotation_angle) > 1e-4 ? BT::NodeStatus::SUCCESS
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
    autonomy::tasks::behavior_tree::plugins::condition::IsTeleopRotateRequested>(
    "IsTeleopRotateRequested");
}

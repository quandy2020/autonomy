/*
 * Copyright 2026 autonomy contributors
 */

#include "autonomy/tasks/behavior_tree/plugins/condition/is_teleop_linear_sign_condition.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

IsTeleopLinearSign::IsTeleopLinearSign(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus IsTeleopLinearSign::tick()
{
  std::string sign = "positive";
  getInput("sign", sign);

  double signed_distance = 0.0;
  if (config().blackboard) {
    config().blackboard->get("teleop_linear_signed", signed_distance);  // NOLINT
  }

  static constexpr double kEpsilon = 1e-4;
  if (sign == "positive") {
    return signed_distance > kEpsilon ? BT::NodeStatus::SUCCESS
                                      : BT::NodeStatus::FAILURE;
  }
  if (sign == "negative") {
    return signed_distance < -kEpsilon ? BT::NodeStatus::SUCCESS
                                     : BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<
    autonomy::tasks::behavior_tree::plugins::condition::IsTeleopLinearSign>(
    "IsTeleopLinearSign");
}

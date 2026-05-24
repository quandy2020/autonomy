/*
 * Copyright 2026 autonomy contributors
 */

#pragma once

#include <string>

#include "behaviortree_cpp/behavior_tree.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

class IsTeleopLinearSign : public BT::ConditionNode
{
public:
  IsTeleopLinearSign(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("sign", "positive|negative")};
  }

  BT::NodeStatus tick() override;
};

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

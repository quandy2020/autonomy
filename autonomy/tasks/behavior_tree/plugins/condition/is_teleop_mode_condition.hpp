/*
 * Copyright 2026 autonomy contributors
 */

#pragma once

#include <string>

#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "behaviortree_cpp/behavior_tree.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

class IsTeleopMode : public BT::ConditionNode
{
public:
  IsTeleopMode(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("mode", "velocity|raw|forward|backward|rotate|sequence")};
  }

  BT::NodeStatus tick() override;
};

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

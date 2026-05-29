/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class IsBatteryChargingCondition : public BT::ConditionNode
{
public:
    IsBatteryChargingCondition(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::ConditionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("battery_topic", "battery")};
    }

    BT::NodeStatus tick() override { return BT::NodeStatus::FAILURE; }

private:
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(IsBatteryChargingCondition, "IsBatteryCharging")

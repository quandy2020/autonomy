/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class IsTeleopModeCondition : public BT::ConditionNode
{
public:
    IsTeleopModeCondition(const std::string& name,
                          const BT::NodeConfiguration& conf)
        : BT::ConditionNode(name, conf) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        bool teleop = false;
        config().blackboard->get("teleop_mode", teleop);
        return teleop ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::IsTeleopModeCondition>(
        "IsTeleopMode");
}

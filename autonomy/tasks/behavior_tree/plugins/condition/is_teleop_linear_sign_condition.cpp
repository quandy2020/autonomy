/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class IsTeleopLinearSignCondition : public BT::ConditionNode
{
public:
    IsTeleopLinearSignCondition(const std::string& name,
                                const BT::NodeConfiguration& conf)
        : BT::ConditionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<int>("sign", 1, "+1 or -1")};
    }

    BT::NodeStatus tick() override {
        int expected = 1;
        getInput("sign", expected);
        int linear_sign = 0;
        config().blackboard->get("teleop_linear_sign", linear_sign);
        return linear_sign == expected ? BT::NodeStatus::SUCCESS
                                       : BT::NodeStatus::FAILURE;
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<
        autonomy::tasks::behavior_tree::IsTeleopLinearSignCondition>(
        "IsTeleopLinearSign");
}

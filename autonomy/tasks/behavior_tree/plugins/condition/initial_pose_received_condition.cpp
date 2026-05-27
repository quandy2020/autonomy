/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class InitialPoseReceivedCondition : public BT::ConditionNode
{
public:
    InitialPoseReceivedCondition(const std::string& name,
                                 const BT::NodeConfiguration& conf)
        : BT::ConditionNode(name, conf) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        bool received = false;
        if (!config().blackboard->get(kBlackboardInitialPoseReceivedKey,
                                      received)) {
            received = true;
        }
        return received ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<
        autonomy::tasks::behavior_tree::InitialPoseReceivedCondition>(
        "InitialPoseReceived");
}

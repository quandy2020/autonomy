/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/decorator_node.h"
namespace autonomy {
namespace tasks {
namespace behavior_tree {

class SingleTrigger : public BT::DecoratorNode
{
public:
    SingleTrigger(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::DecoratorNode(name, conf) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        if (!BT::isStatusActive(status())) {
            first_time_ = true;
        }
        setStatus(BT::NodeStatus::RUNNING);
        if (!first_time_) {
            return BT::NodeStatus::FAILURE;
        }
        const BT::NodeStatus child_state = child_node_->executeTick();
        switch (child_state) {
            case BT::NodeStatus::SKIPPED:
            case BT::NodeStatus::RUNNING:
                return child_state;
            case BT::NodeStatus::SUCCESS:
            case BT::NodeStatus::FAILURE:
                first_time_ = false;
                return child_state;
            default:
                first_time_ = false;
                return BT::NodeStatus::FAILURE;
        }
    }

private:
    bool first_time_{true};
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(SingleTrigger, "SingleTrigger")

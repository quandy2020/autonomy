/*
 * Copyright 2026 The Openbot Authors
 *
 * Nav2-compatible RecoveryNode: tick child[0]; on FAILURE run child[1] and
 * retry child[0] up to number_of_retries times.
 */

#include <string>

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "behaviortree_cpp/control_node.h"

namespace autonomy::task::plugins {

class RecoveryNode : public BT::ControlNode
{
public:
    RecoveryNode(const std::string& name, const BT::NodeConfig& config)
        : BT::ControlNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<int>("number_of_retries", 1,
                                   "max successful recovery cycles")};
    }

    void halt() override
    {
        ControlNode::halt();
        retry_count_ = 0;
        current_child_idx_ = 0;
    }

private:
    unsigned current_child_idx_{0};
    int number_of_retries_{1};
    int retry_count_{0};

    BT::NodeStatus tick() override
    {
        getInput("number_of_retries", number_of_retries_);
        const unsigned children_count =
            static_cast<unsigned>(children_nodes_.size());
        if (children_count != 2) {
            throw BT::BehaviorTreeException(
                "RecoveryNode '" + name() + "' must only have 2 children.");
        }

        setStatus(BT::NodeStatus::RUNNING);

        while (current_child_idx_ < children_count &&
               retry_count_ <= number_of_retries_) {
            BT::TreeNode* child_node = children_nodes_[current_child_idx_];
            const BT::NodeStatus child_status = child_node->executeTick();

            if (current_child_idx_ == 0) {
                switch (child_status) {
                case BT::NodeStatus::SKIPPED:
                    halt();
                    return BT::NodeStatus::SKIPPED;
                case BT::NodeStatus::SUCCESS:
                    ControlNode::haltChild(1);
                    halt();
                    return BT::NodeStatus::SUCCESS;
                case BT::NodeStatus::RUNNING:
                    return BT::NodeStatus::RUNNING;
                case BT::NodeStatus::FAILURE:
                    if (retry_count_ < number_of_retries_) {
                        ControlNode::haltChild(0);
                        current_child_idx_++;
                        break;
                    }
                    halt();
                    return BT::NodeStatus::FAILURE;
                default:
                    throw BT::LogicError("A child node must never return IDLE");
                }
            } else if (current_child_idx_ == 1) {
                switch (child_status) {
                case BT::NodeStatus::SKIPPED:
                    current_child_idx_ = 0;
                    ControlNode::haltChild(1);
                    return BT::NodeStatus::FAILURE;
                case BT::NodeStatus::RUNNING:
                    return child_status;
                case BT::NodeStatus::SUCCESS:
                    ControlNode::haltChild(1);
                    retry_count_++;
                    current_child_idx_ = 0;
                    break;
                case BT::NodeStatus::FAILURE:
                    halt();
                    return BT::NodeStatus::FAILURE;
                default:
                    throw BT::LogicError("A child node must never return IDLE");
                }
            }
        }

        halt();
        return BT::NodeStatus::FAILURE;
    }
};

}  // namespace autonomy::task::plugins

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::RecoveryNode>(
        "RecoveryNode");
}

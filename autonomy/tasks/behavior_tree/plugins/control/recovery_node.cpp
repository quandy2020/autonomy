/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 * Ported from nav2_behavior_tree RecoveryNode.
 */

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/control_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class RecoveryNode : public BT::ControlNode
{
public:
    RecoveryNode(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::ControlNode(name, conf),
          current_child_idx_(0),
          number_of_retries_(1),
          retry_count_(0) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<int>("number_of_retries", 1, "Retries")};
    }

    BT::NodeStatus tick() override {
        getInput("number_of_retries", number_of_retries_);
        const unsigned children_count = children_nodes_.size();
        if (children_count != 2) {
            throw BT::BehaviorTreeException("RecoveryNode must have 2 children.");
        }
        setStatus(BT::NodeStatus::RUNNING);
        while (current_child_idx_ < children_count &&
               retry_count_ <= number_of_retries_) {
            auto* child = children_nodes_[current_child_idx_];
            const BT::NodeStatus child_status = child->executeTick();
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
                        throw BT::LogicError("Invalid child status");
                }
            } else {
                switch (child_status) {
                    case BT::NodeStatus::SKIPPED:
                        current_child_idx_ = 0;
                        ControlNode::haltChild(1);
                        return BT::NodeStatus::FAILURE;
                    case BT::NodeStatus::RUNNING:
                        return BT::NodeStatus::RUNNING;
                    case BT::NodeStatus::SUCCESS:
                        ControlNode::haltChild(1);
                        retry_count_++;
                        current_child_idx_ = 0;
                        break;
                    case BT::NodeStatus::FAILURE:
                        halt();
                        return BT::NodeStatus::FAILURE;
                    default:
                        throw BT::LogicError("Invalid child status");
                }
            }
        }
        halt();
        return BT::NodeStatus::FAILURE;
    }

    void halt() override {
        ControlNode::halt();
        retry_count_ = 0;
        current_child_idx_ = 0;
    }

private:
    size_t current_child_idx_;
    int number_of_retries_;
    int retry_count_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(RecoveryNode, "RecoveryNode")

/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 * Ported from nav2_behavior_tree RoundRobinNode.
 */

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/control_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class RoundRobinNode : public BT::ControlNode
{
public:
    RoundRobinNode(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::ControlNode(name, conf),
          current_child_idx_(0),
          num_failed_children_(0) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        const auto num_children = children_nodes_.size();
        setStatus(BT::NodeStatus::RUNNING);
        unsigned num_skipped = 0;
        while (num_failed_children_ + num_skipped < num_children) {
            auto* child = children_nodes_[current_child_idx_];
            const BT::NodeStatus child_status = child->executeTick();
            if (child_status != BT::NodeStatus::RUNNING) {
                if (++current_child_idx_ == num_children) {
                    current_child_idx_ = 0;
                }
            }
            switch (child_status) {
                case BT::NodeStatus::SUCCESS:
                    num_failed_children_ = 0;
                    ControlNode::haltChildren();
                    return BT::NodeStatus::SUCCESS;
                case BT::NodeStatus::FAILURE:
                    num_failed_children_++;
                    break;
                case BT::NodeStatus::SKIPPED:
                    num_skipped++;
                    break;
                case BT::NodeStatus::RUNNING:
                    return BT::NodeStatus::RUNNING;
                default:
                    throw BT::LogicError("Invalid child status");
            }
        }
        const bool all_skipped = (num_skipped == num_children);
        halt();
        return all_skipped ? BT::NodeStatus::SKIPPED : BT::NodeStatus::FAILURE;
    }

    void halt() override {
        ControlNode::halt();
        current_child_idx_ = 0;
        num_failed_children_ = 0;
    }

private:
    size_t current_child_idx_;
    size_t num_failed_children_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(RoundRobinNode, "RoundRobin")

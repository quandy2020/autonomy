/*
 * Copyright 2026 The Openbot Authors
 *
 * Nav2-compatible RoundRobin control node.
 */

#include <string>

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "behaviortree_cpp/control_node.h"

namespace autonomy::task::plugins {

class RoundRobin : public BT::ControlNode
{
public:
    RoundRobin(const std::string& name, const BT::NodeConfig& config)
        : BT::ControlNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        // Default true so LocalMotionCycle keeps cycling in navigate_to_pose.
        return {BT::InputPort<bool>("wrap_around", true,
                                    "wrap to first child after last")};
    }

    void halt() override
    {
        ControlNode::halt();
        current_child_idx_ = 0;
        num_failed_children_ = 0;
    }

private:
    size_t current_child_idx_{0};
    unsigned num_failed_children_{0};
    bool wrap_around_{true};

    BT::NodeStatus tick() override
    {
        getInput("wrap_around", wrap_around_);
        const auto num_children = children_nodes_.size();
        if (num_children == 0) {
            return BT::NodeStatus::SUCCESS;
        }

        setStatus(BT::NodeStatus::RUNNING);
        unsigned num_skipped_children = 0;

        while (num_failed_children_ + num_skipped_children < num_children) {
            BT::TreeNode* child_node = children_nodes_[current_child_idx_];
            const BT::NodeStatus child_status = child_node->executeTick();

            if (child_status != BT::NodeStatus::RUNNING) {
                if (++current_child_idx_ == num_children) {
                    if (wrap_around_) {
                        current_child_idx_ = 0;
                    } else {
                        if (child_status == BT::NodeStatus::SKIPPED) {
                            num_skipped_children++;
                        } else if (child_status == BT::NodeStatus::FAILURE) {
                            num_failed_children_++;
                        }
                        break;
                    }
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
                num_skipped_children++;
                break;
            case BT::NodeStatus::RUNNING:
                return BT::NodeStatus::RUNNING;
            default:
                throw BT::LogicError("Invalid status return from BT node");
            }
        }

        const bool all_skipped = (num_skipped_children == num_children);
        halt();
        return all_skipped ? BT::NodeStatus::SKIPPED : BT::NodeStatus::FAILURE;
    }
};

}  // namespace autonomy::task::plugins

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::RoundRobin>("RoundRobin");
}

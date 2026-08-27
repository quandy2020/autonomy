/*
 * Copyright 2026 The Openbot Authors
 *
 * Nav2-style PipelineSequence: keep ticking earlier children (e.g. RateController
 * replan) while a later child (FollowPath) is RUNNING.
 */

#include <string>

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "behaviortree_cpp/control_node.h"

namespace autonomy::task::plugins {

class PipelineSequence : public BT::ControlNode
{
public:
    PipelineSequence(const std::string& name, const BT::NodeConfig& config)
        : BT::ControlNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    void halt() override
    {
        current_child_idx_ = 0;
        ControlNode::halt();
    }

private:
    size_t current_child_idx_{0};

    BT::NodeStatus tick() override
    {
        const size_t children_count = children_nodes_.size();
        if (children_count == 0) {
            return BT::NodeStatus::SUCCESS;
        }

        // Re-tick children [0..current] so RateController keeps planning while
        // FollowPath is RUNNING. Never advance past the last child in a way
        // that expands the for-bound (that OOB-crashed autonomy.task on goal
        // reached and triggered launch respawn).
        while (current_child_idx_ < children_count) {
            for (size_t i = 0; i <= current_child_idx_; ++i) {
                const BT::NodeStatus status = children_nodes_[i]->executeTick();
                switch (status) {
                case BT::NodeStatus::FAILURE:
                    haltChildren();
                    current_child_idx_ = 0;
                    return BT::NodeStatus::FAILURE;
                case BT::NodeStatus::RUNNING:
                    return BT::NodeStatus::RUNNING;
                case BT::NodeStatus::SUCCESS:
                    if (i != current_child_idx_) {
                        break;
                    }
                    if (current_child_idx_ + 1 >= children_count) {
                        haltChildren();
                        current_child_idx_ = 0;
                        return BT::NodeStatus::SUCCESS;
                    }
                    ++current_child_idx_;
                    break;
                default:
                    break;
                }
            }
        }

        haltChildren();
        current_child_idx_ = 0;
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace autonomy::task::plugins

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::PipelineSequence>(
        "PipelineSequence");
}

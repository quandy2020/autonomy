/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 * Ported from nav2_behavior_tree PipelineSequence.
 */

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/control_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class PipelineSequence : public BT::ControlNode
{
public:
    PipelineSequence(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::ControlNode(name, conf), last_child_ticked_(0) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        unsigned skipped = 0;
        for (std::size_t i = 0; i < children_nodes_.size(); ++i) {
            auto status = children_nodes_[i]->executeTick();
            switch (status) {
                case BT::NodeStatus::FAILURE:
                    ControlNode::haltChildren();
                    last_child_ticked_ = 0;
                    return status;
                case BT::NodeStatus::SKIPPED:
                    skipped++;
                    break;
                case BT::NodeStatus::SUCCESS:
                    break;
                case BT::NodeStatus::RUNNING:
                    if (i >= last_child_ticked_) {
                        last_child_ticked_ = i;
                    }
                    return status;
                default:
                    break;
            }
        }
        ControlNode::haltChildren();
        last_child_ticked_ = 0;
        return skipped == children_nodes_.size() ? BT::NodeStatus::SKIPPED
                                                 : BT::NodeStatus::SUCCESS;
    }

    void halt() override {
        ControlNode::haltChildren();
        last_child_ticked_ = 0;
        resetStatus();
    }

private:
    std::size_t last_child_ticked_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(PipelineSequence, "PipelineSequence")

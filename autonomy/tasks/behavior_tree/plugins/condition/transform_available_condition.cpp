/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class TransformAvailableCondition : public BT::ConditionNode
{
public:
    TransformAvailableCondition(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::ConditionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("child", "{robot_base_frame}"),
            BT::InputPort<std::string>("parent", "{global_frame}"),
        };
    }

    BT::NodeStatus tick() override {
        if (was_found_) {
            return BT::NodeStatus::SUCCESS;
        }
        auto ctx = GetContext(config());
        if (!ctx || !ctx->tf_buffer) {
            return BT::NodeStatus::FAILURE;
        }
        std::string child;
        std::string parent;
        getInput("child", child);
        getInput("parent", parent);
        if (child.empty()) {
            child = ctx->options.robot_base_frame();
        }
        if (parent.empty()) {
            parent = ctx->options.global_frame();
        }
        std::string error;
        was_found_ = static_cast<transform::tf2::BufferCore&>(*ctx->tf_buffer)
                         .canTransform(parent, child, 0ULL, &error);
        return was_found_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    bool was_found_{false};
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(TransformAvailableCondition, "TransformAvailable")

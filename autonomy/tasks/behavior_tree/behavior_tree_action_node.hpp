/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include "autonomy/tasks/behavior_tree/bt_plugin_common.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/**
 * @brief Base for long-running BT actions that call in-process servers each tick.
 */
class BehaviorTreeActionNode : public BT::ActionNodeBase
{
public:
    BehaviorTreeActionNode(const std::string& xml_tag_name,
                           const BT::NodeConfiguration& conf)
        : BT::ActionNodeBase(xml_tag_name, conf) {}

    BT::NodeStatus tick() override final {
        if (!BT::isStatusActive(status())) {
            onStart();
            setStatus(BT::NodeStatus::RUNNING);
        }
        if (WaitIfPaused(config())) {
            return BT::NodeStatus::RUNNING;
        }
        if (IsCancelRequested(config())) {
            onHalted();
            return BT::NodeStatus::FAILURE;
        }
        return onRunning();
    }

    void halt() override {
        onHalted();
        resetStatus();
    }

protected:
    virtual void onStart() {}
    virtual BT::NodeStatus onRunning() = 0;
    virtual void onHalted() {}
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

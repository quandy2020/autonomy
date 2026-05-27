/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <functional>
#include <memory>
#include <string>

#include "autonomy/tasks/behavior_tree/utils.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/**
 * @brief Base for long-running BT actions that call in-process servers each tick.
 */
class ActionNode : public BT::ActionNodeBase
{
public:
    ActionNode(const std::string& xml_tag_name,
                           const BT::NodeConfiguration& conf)
        : BT::ActionNodeBase(xml_tag_name, conf) {}

    BT::NodeStatus tick() override final {
        if (!BT::isStatusActive(status())) {
            ctx_ = GetContext(config());
            onStart();
            setStatus(BT::NodeStatus::RUNNING);
        }
        if (WaitIfPaused(config())) {
            return BT::NodeStatus::RUNNING;
        }
        if (IsCancelRequested(config())) {
            onHalted();
            ctx_.reset();
            return BT::NodeStatus::FAILURE;
        }
        return onRunning();
    }

    void halt() override {
        onHalted();
        ctx_.reset();
        resetStatus();
    }

protected:
    /** Cached BT context for this activation (refreshed on each onStart). */
    std::shared_ptr<BehaviorTreeContext> Context() const { return ctx_; }

    virtual void onStart() {}
    virtual BT::NodeStatus onRunning() = 0;
    virtual void onHalted() {}

private:
    std::shared_ptr<BehaviorTreeContext> ctx_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

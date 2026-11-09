/* Copyright 2026 The Navigation Authors (navigation) */
#pragma once

#include <behaviortree_cpp/control_node.h>
#include "navigation/behavior/nodes/bt_node_base.hpp"

namespace navigation::behavior {

/** @brief Re-tick upstream children while a downstream action is running. */
class PipelineSequence final : public BT::ControlNode
{
public:
    using BT::ControlNode::ControlNode;
    static BT::PortsList providedPorts() {
        return {};
    }
    BT::NodeStatus tick() override;
    void halt() override {
        last_running_ = 0;
        BT::ControlNode::halt();
    }

private:
    std::size_t last_running_{0};
};

/** @brief Two-child navigation/recovery loop with a persistent attempt budget.
 */
class RecoveryNode final : public BT::ControlNode
{
public:
    using BT::ControlNode::ControlNode;
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override {
        recovering_ = false;
        retries_ = 0;
        BT::ControlNode::halt();
    }

private:
    bool recovering_{false};
    std::uint32_t retries_{0};
};

class WaitForGoalReachedNode final : public NavigationStatefulAction
{
public:
    using NavigationStatefulAction::NavigationStatefulAction;
    static BT::PortsList providedPorts();

protected:
    BT::NodeStatus OnFirstTick(NodeContext& context) override;
    BT::NodeStatus OnExecute(NodeContext& context) override;

private:
    std::int64_t started_ns_{0};
};

/** @brief Request once, wait for acceptance, then wait for the Runner outcome.
 */
class RecoveryExecutorNode final : public NavigationStatefulAction
{
public:
    using NavigationStatefulAction::NavigationStatefulAction;

protected:
    BT::NodeStatus OnFirstTick(NodeContext& context) override;
    BT::NodeStatus OnExecute(NodeContext& context) override;

private:
    std::uint32_t expected_count_{0};
    std::int64_t requested_ns_{0}, last_send_ns_{0};
};
}  // namespace navigation::behavior

/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file request_operator_assist_node.hpp
 * @brief BehaviorTree action: request operator assistance once.
 */

#pragma once

#include <string>
#include <string_view>

#include "navigation/behavior/nodes/bt_node_base.hpp"

namespace navigation
{
namespace behavior
{

/** @brief Sets the operator-assist latch and invokes the callback once. */
class RequestOperatorAssistNode : public NavigationSyncAction
{
public:
    RequestOperatorAssistNode();
    RequestOperatorAssistNode(const std::string& name, const BT::NodeConfig& config);

    /** @brief Construct with a default reason for unit tests. */
    explicit RequestOperatorAssistNode(std::string_view reason);

    bool Tick(NodeContext& context) const;

    static BT::PortsList providedPorts();

protected:
    BT::NodeStatus OnExecute(NodeContext& context) override;

private:
    std::string reason_{"recovery_exhausted"};
};

}  // namespace behavior
}  // namespace navigation

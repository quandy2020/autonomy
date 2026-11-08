/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/behavior/nodes/request_operator_assist_node.hpp"

#include <utility>

namespace navigation
{
namespace behavior
{

RequestOperatorAssistNode::RequestOperatorAssistNode()
    : NavigationSyncAction("RequestOperatorAssist", BT::NodeConfig{})
{
}

RequestOperatorAssistNode::RequestOperatorAssistNode(
    const std::string& name, const BT::NodeConfig& config)
    : NavigationSyncAction(name, config)
{
}

RequestOperatorAssistNode::RequestOperatorAssistNode(std::string_view reason)
    : NavigationSyncAction("RequestOperatorAssist", BT::NodeConfig{}),
      reason_(reason)
{
}

bool RequestOperatorAssistNode::Tick(NodeContext& context) const
{
    if (context.assist_requested)
    {
        return false;
    }
    if (context.request_assist)
    {
        context.request_assist(reason_);
    }
    context.assist_requested = true;
    return true;
}

BT::PortsList RequestOperatorAssistNode::providedPorts()
{
    return {BT::InputPort<std::string>(
        "reason", "recovery_exhausted", "operator-assistance reason")};
}

BT::NodeStatus RequestOperatorAssistNode::OnExecute(NodeContext& context)
{
    std::string reason;
    if (getInput("reason", reason))
    {
        reason_ = reason;
    }
    return Tick(context) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace behavior
}  // namespace navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<navigation::behavior::RequestOperatorAssistNode>(
        "RequestOperatorAssist");
}

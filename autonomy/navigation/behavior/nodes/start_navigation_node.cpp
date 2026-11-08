/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/behavior/nodes/start_navigation_node.hpp"

namespace navigation
{
namespace behavior
{

StartNavigationNode::StartNavigationNode()
    : NavigationSyncAction("StartNavigation", BT::NodeConfig{})
{
}

StartNavigationNode::StartNavigationNode(const std::string& name,
                                         const BT::NodeConfig& config)
    : NavigationSyncAction(name, config)
{
}

bool StartNavigationNode::Tick(NodeContext& context) const
{
    if (context.session == nullptr)
    {
        return false;
    }
    return context.session->SetGoal(context.goal);
}

BT::NodeStatus StartNavigationNode::OnExecute(NodeContext& context)
{
    return Tick(context) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace behavior
}  // namespace navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<navigation::behavior::StartNavigationNode>(
        "StartNavigation");
}

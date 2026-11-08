/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/behavior/nodes/check_goal_reachable_node.hpp"

namespace navigation
{
namespace behavior
{

CheckGoalReachableNode::CheckGoalReachableNode()
    : NavigationCondition("CheckGoalReachable", BT::NodeConfig{})
{
}

CheckGoalReachableNode::CheckGoalReachableNode(const std::string& name,
                                               const BT::NodeConfig& config)
    : NavigationCondition(name, config)
{
}

bool CheckGoalReachableNode::Tick(const NodeContext& context) const
{
    return context.goal_reachable;
}

BT::NodeStatus CheckGoalReachableNode::OnEvaluate(const NodeContext& context)
{
    return Tick(context) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace behavior
}  // namespace navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<navigation::behavior::CheckGoalReachableNode>(
        "CheckGoalReachable");
}

/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/behavior/nodes/check_module_health_node.hpp"

namespace navigation
{
namespace behavior
{

CheckModuleHealthNode::CheckModuleHealthNode()
    : NavigationCondition("CheckModuleHealth", BT::NodeConfig{})
{
}

CheckModuleHealthNode::CheckModuleHealthNode(const std::string& name,
                                             const BT::NodeConfig& config)
    : NavigationCondition(name, config)
{
}

bool CheckModuleHealthNode::Tick(const NodeContext& context) const
{
    return context.module_healthy;
}

BT::NodeStatus CheckModuleHealthNode::OnEvaluate(const NodeContext& context)
{
    return Tick(context) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace behavior
}  // namespace navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<navigation::behavior::CheckModuleHealthNode>(
        "CheckModuleHealth");
}

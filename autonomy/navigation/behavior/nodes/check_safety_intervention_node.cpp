/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/behavior/nodes/check_safety_intervention_node.hpp"

namespace navigation
{
namespace behavior
{

CheckSafetyInterventionNode::CheckSafetyInterventionNode()
    : NavigationCondition("CheckSafetyIntervention", BT::NodeConfig{})
{
}

CheckSafetyInterventionNode::CheckSafetyInterventionNode(
    const std::string& name, const BT::NodeConfig& config)
    : NavigationCondition(name, config)
{
}

bool CheckSafetyInterventionNode::Tick(const NodeContext& context) const
{
    return !context.safety_intervention;
}

BT::NodeStatus CheckSafetyInterventionNode::OnEvaluate(const NodeContext& context)
{
    return Tick(context) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace behavior
}  // namespace navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<navigation::behavior::CheckSafetyInterventionNode>(
        "CheckSafetyIntervention");
}

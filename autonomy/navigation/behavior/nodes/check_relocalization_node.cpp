/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/behavior/nodes/check_relocalization_node.hpp"

namespace navigation
{
namespace behavior
{

CheckRelocalizationNode::CheckRelocalizationNode()
    : NavigationCondition("CheckRelocalization", BT::NodeConfig{})
{
}

CheckRelocalizationNode::CheckRelocalizationNode(const std::string& name,
                                                 const BT::NodeConfig& config)
    : NavigationCondition(name, config)
{
}

bool CheckRelocalizationNode::Tick(const NodeContext& context) const
{
    return context.relocalized;
}

BT::NodeStatus CheckRelocalizationNode::OnEvaluate(const NodeContext& context)
{
    return Tick(context) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace behavior
}  // namespace navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<navigation::behavior::CheckRelocalizationNode>(
        "CheckRelocalization");
}

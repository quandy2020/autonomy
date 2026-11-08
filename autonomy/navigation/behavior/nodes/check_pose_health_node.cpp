/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/behavior/nodes/check_pose_health_node.hpp"

namespace navigation
{
namespace behavior
{

CheckPoseHealthNode::CheckPoseHealthNode()
    : NavigationCondition("CheckPoseHealth", BT::NodeConfig{})
{
}

CheckPoseHealthNode::CheckPoseHealthNode(const std::string& name,
                                         const BT::NodeConfig& config)
    : NavigationCondition(name, config)
{
}

bool CheckPoseHealthNode::Tick(const NodeContext& context) const
{
    return context.pose_valid;
}

BT::NodeStatus CheckPoseHealthNode::OnEvaluate(const NodeContext& context)
{
    return Tick(context) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace behavior
}  // namespace navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<navigation::behavior::CheckPoseHealthNode>("CheckPoseHealth");
}

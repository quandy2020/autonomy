/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file check_pose_health_node.hpp
 * @brief BehaviorTree condition: pose fresh and continuous.
 */

#pragma once

#include <string>

#include "navigation/behavior/nodes/bt_node_base.hpp"

namespace navigation
{
namespace behavior
{

/** @brief Returns SUCCESS when the tracked pose is valid. */
class CheckPoseHealthNode : public NavigationCondition
{
public:
    CheckPoseHealthNode();
    CheckPoseHealthNode(const std::string& name, const BT::NodeConfig& config);

    /** @brief Plain evaluation used by unit tests. */
    bool Tick(const NodeContext& context) const;

protected:
    BT::NodeStatus OnEvaluate(const NodeContext& context) override;
};

}  // namespace behavior
}  // namespace navigation

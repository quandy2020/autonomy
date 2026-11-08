/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file check_goal_reachable_node.hpp
 * @brief BehaviorTree condition: goal is reachable.
 */

#pragma once

#include <string>

#include "navigation/behavior/nodes/bt_node_base.hpp"

namespace navigation
{
namespace behavior
{

/** @brief Returns SUCCESS when the planner accepts the goal. */
class CheckGoalReachableNode : public NavigationCondition
{
public:
    CheckGoalReachableNode();
    CheckGoalReachableNode(const std::string& name, const BT::NodeConfig& config);

    bool Tick(const NodeContext& context) const;

protected:
    BT::NodeStatus OnEvaluate(const NodeContext& context) override;
};

}  // namespace behavior
}  // namespace navigation

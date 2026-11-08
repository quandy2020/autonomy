/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file start_navigation_node.hpp
 * @brief BehaviorTree action: submit the blackboard goal.
 */

#pragma once

#include <string>

#include "navigation/behavior/nodes/bt_node_base.hpp"

namespace navigation
{
namespace behavior
{

/** @brief Calls NavigationSession::SetGoal once. */
class StartNavigationNode : public NavigationSyncAction
{
public:
    StartNavigationNode();
    StartNavigationNode(const std::string& name, const BT::NodeConfig& config);

    bool Tick(NodeContext& context) const;

protected:
    BT::NodeStatus OnExecute(NodeContext& context) override;
};

}  // namespace behavior
}  // namespace navigation

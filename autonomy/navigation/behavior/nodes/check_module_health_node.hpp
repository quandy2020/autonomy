/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file check_module_health_node.hpp
 * @brief BehaviorTree condition: navigation/safety/memory healthy.
 */

#pragma once

#include <string>

#include "navigation/behavior/nodes/bt_node_base.hpp"

namespace navigation
{
namespace behavior
{

/** @brief Returns SUCCESS when all required modules are healthy. */
class CheckModuleHealthNode : public NavigationCondition
{
public:
    CheckModuleHealthNode();
    CheckModuleHealthNode(const std::string& name, const BT::NodeConfig& config);

    bool Tick(const NodeContext& context) const;

protected:
    BT::NodeStatus OnEvaluate(const NodeContext& context) override;
};

}  // namespace behavior
}  // namespace navigation

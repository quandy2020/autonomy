/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file check_safety_intervention_node.hpp
 * @brief BehaviorTree condition: safety layer has not intervened.
 */

#pragma once

#include <string>

#include "navigation/behavior/nodes/bt_node_base.hpp"

namespace navigation
{
namespace behavior
{

/** @brief Returns SUCCESS when safety has not taken over. */
class CheckSafetyInterventionNode : public NavigationCondition
{
public:
    CheckSafetyInterventionNode();
    CheckSafetyInterventionNode(const std::string& name,
                                const BT::NodeConfig& config);

    bool Tick(const NodeContext& context) const;

protected:
    BT::NodeStatus OnEvaluate(const NodeContext& context) override;
};

}  // namespace behavior
}  // namespace navigation

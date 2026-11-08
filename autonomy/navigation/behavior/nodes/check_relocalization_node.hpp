/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file check_relocalization_node.hpp
 * @brief BehaviorTree condition: pose is relocalized.
 */

#pragma once

#include <string>

#include "navigation/behavior/nodes/bt_node_base.hpp"

namespace navigation
{
namespace behavior
{

/** @brief Returns SUCCESS when localization is valid. */
class CheckRelocalizationNode : public NavigationCondition
{
public:
    CheckRelocalizationNode();
    CheckRelocalizationNode(const std::string& name,
                            const BT::NodeConfig& config);

    bool Tick(const NodeContext& context) const;

protected:
    BT::NodeStatus OnEvaluate(const NodeContext& context) override;
};

}  // namespace behavior
}  // namespace navigation

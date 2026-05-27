/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include "autonomy/tasks/behavior_tree/bt_plugin_common.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/** @brief Synchronous BT node that completes in one tick (e.g. ClearCostmap). */
class BehaviorTreeServiceNode : public BT::SyncActionNode
{
public:
    BehaviorTreeServiceNode(const std::string& xml_tag_name,
                            const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(xml_tag_name, conf) {}

    BT::NodeStatus tick() override final {
        if (IsCancelRequested(config())) {
            return BT::NodeStatus::FAILURE;
        }
        onTick();
        return onService();
    }

protected:
    virtual void onTick() {}
    virtual BT::NodeStatus onService() = 0;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#ifndef AUTONOMY_TASKS_BEHAVIOR_TREE_BT_SERVICE_NODE_H_
#define AUTONOMY_TASKS_BEHAVIOR_TREE_BT_SERVICE_NODE_H_

#include <string>

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/**
 * @brief Synchronous BT action node (nav2 BtServiceNode analogue).
 *
 * Subclasses implement onService() for one-shot work. onTick() runs before
 * onService() when the node is ticked.
 */
class BtServiceNode : public BT::SyncActionNode
{
public:
    BtServiceNode(const std::string& xml_tag_name,
                  const BT::NodeConfiguration& conf);

    BT::NodeStatus tick() override final;

protected:
    /** Optional hook before onService(); default is no-op. */
    virtual void onTick() {}

    /**
     * @brief Perform the service action.
     * @return SUCCESS or FAILURE for the BT tick.
     */
    virtual BT::NodeStatus onService() = 0;
};

inline BtServiceNode::BtServiceNode(const std::string& xml_tag_name,
                                    const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(xml_tag_name, conf) {}

inline BT::NodeStatus BtServiceNode::tick() {
    if (IsCancelRequested(config())) {
        return BT::NodeStatus::FAILURE;
    }
    onTick();
    return onService();
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#endif  // AUTONOMY_TASKS_BEHAVIOR_TREE_BT_SERVICE_NODE_H_

/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/bt_cancel_action_node.hpp"
#include "autonomy/tasks/navigators/action_type.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class DriveOnHeadingCancel : public BtCancelActionNode<DriveOnHeadingActionTraits>
{
public:
    DriveOnHeadingCancel(const std::string& name, const BT::NodeConfiguration& conf)
        : BtCancelActionNode(name, kDriveOnHeadingActionName, conf) {}
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(DriveOnHeadingCancel, "CancelDriveOnHeading")

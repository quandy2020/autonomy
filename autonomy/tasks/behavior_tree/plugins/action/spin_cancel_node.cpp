/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/bt_cancel_action_node.hpp"
#include "autonomy/tasks/navigators/action_type.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class SpinCancel : public BtCancelActionNode<SpinActionTraits>
{
public:
    SpinCancel(const std::string& name, const BT::NodeConfiguration& conf)
        : BtCancelActionNode(name, kSpinActionName, conf) {}
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(SpinCancel, "CancelSpin")

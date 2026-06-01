/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "autonomy/tasks/constants.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class IsPathValidCondition : public BT::ConditionNode
{
public:
    IsPathValidCondition(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::ConditionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<commsgs::planning_msgs::Path>("path")};
    }

    BT::NodeStatus tick() override {
        commsgs::planning_msgs::Path path;
        getInput("path", path);
        if (path.poses.empty()) {
            getInput(kBlackboardPathKey, path);
        }
        if (path.poses.size() < kMinPathPoses) {
            return BT::NodeStatus::FAILURE;
        }

        const auto ctx = GetBtContext(config().blackboard);
        if (!ctx || !ctx->planner) {
            return BT::NodeStatus::FAILURE;
        }

        return ctx->planner->IsPathValid(path) ? BT::NodeStatus::SUCCESS
                                              : BT::NodeStatus::FAILURE;
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(IsPathValidCondition, "IsPathValid")

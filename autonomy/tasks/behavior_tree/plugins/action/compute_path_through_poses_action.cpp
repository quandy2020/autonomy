/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include <vector>

#include "autonomy/tasks/behavior_tree/node_utils.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class ComputePathThroughPosesAction : public BT::SyncActionNode
{
public:
    ComputePathThroughPosesAction(const std::string& name,
                                  const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::vector<commsgs::geometry_msgs::PoseStamped>>(
                "goals"),
            BT::InputPort<std::string>("planner_id", "", "Planner id"),
            BT::OutputPort<commsgs::planning_msgs::Path>("path"),
            BT::OutputPort<uint16_t>("error_code_id"),
        };
    }

    BT::NodeStatus tick() override {
        if (IsCancelRequested(config())) {
            return BT::NodeStatus::FAILURE;
        }
        return RunGetPath<ComputePathThroughPosesAction,
                          std::vector<commsgs::geometry_msgs::PoseStamped>>(
            *this, GetContext(config()), "goals", kBlackboardGoalsKey,
            [](BehaviorTreeContext& ctx, const auto& start, const auto& goals,
               const std::string& planner_id) {
                return ctx.planner->ComputePathThroughPoses(
                    start, goals, planner_id, ctx.CancelChecker());
            });
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(ComputePathThroughPosesAction,
                          "ComputePathThroughPoses")

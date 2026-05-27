/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/node_utils.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class ComputePathToPoseAction : public BT::SyncActionNode
{
public:
    ComputePathToPoseAction(const std::string& name,
                            const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        BT::PortsList ports = {
            BT::InputPort<commsgs::geometry_msgs::PoseStamped>("goal"),
            BT::InputPort<std::string>("planner_id", "", "Planner id"),
            BT::OutputPort<commsgs::planning_msgs::Path>("path"),
        };
        return AppendErrorOutcomePorts(ports);
    }

    BT::NodeStatus tick() override {
        if (IsCancelRequested(config())) {
            return BT::NodeStatus::FAILURE;
        }
        return RunGetPath<ComputePathToPoseAction,
                          commsgs::geometry_msgs::PoseStamped>(
            *this, GetContext(config()), "goal", kBlackboardGoalKey,
            [](BehaviorTreeContext& ctx, const auto& start, const auto& goal,
               const std::string& planner_id) {
                return ctx.planner->ComputePathToPose(start, goal, planner_id,
                                                      ctx.CancelChecker());
            });
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(ComputePathToPoseAction, "ComputePathToPose")

/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class GoalReachedCondition : public BT::ConditionNode
{
public:
    GoalReachedCondition(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::ConditionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<commsgs::geometry_msgs::PoseStamped>("goal"),
            BT::InputPort<double>("goal_reached_tol", 0.25, "Tolerance m"),
            BT::InputPort<double>("transform_tolerance", 0.1, "TF tol"),
        };
    }

    BT::NodeStatus tick() override {
        auto ctx = GetContext(config());
        if (!ctx) {
            return BT::NodeStatus::FAILURE;
        }
        commsgs::geometry_msgs::PoseStamped goal;
        if (!GetInputOrBlackboard(*this, config(), "goal", kBlackboardGoalKey, goal)) {
            return BT::NodeStatus::FAILURE;
        }
        double tol = 0.25;
        getInput("goal_reached_tol", tol);
        if (tol <= 0.0) {
            config().blackboard->get(kBlackboardGoalReachedTolKey, tol);
        }
        commsgs::geometry_msgs::PoseStamped current;
        if (!GetGlobalStartPose(*ctx, current)) {
            return BT::NodeStatus::FAILURE;
        }
        const double dx = current.pose.position.x - goal.pose.position.x;
        const double dy = current.pose.position.y - goal.pose.position.y;
        return std::hypot(dx, dy) <= tol ? BT::NodeStatus::SUCCESS
                                         : BT::NodeStatus::FAILURE;
    }

private:
    commsgs::geometry_msgs::PoseStamped goal_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(GoalReachedCondition, "GoalReached")

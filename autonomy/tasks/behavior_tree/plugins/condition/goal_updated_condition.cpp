/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class GoalUpdatedCondition : public BT::ConditionNode
{
public:
    GoalUpdatedCondition(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::ConditionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<commsgs::geometry_msgs::PoseStamped>("goal"),
            BT::InputPort<std::vector<commsgs::geometry_msgs::PoseStamped>>(
                "goals"),
        };
    }

    BT::NodeStatus tick() override {
        if (!BT::isStatusActive(status())) {
            GetInputOrBlackboard(*this, config(), "goal", kBlackboardGoalKey, goal_);
            GetInputOrBlackboard(*this, config(), "goals", kBlackboardGoalsKey, goals_);
            return BT::NodeStatus::FAILURE;
        }
        commsgs::geometry_msgs::PoseStamped current_goal;
        std::vector<commsgs::geometry_msgs::PoseStamped> current_goals;
        GetInputOrBlackboard(*this, config(), "goal", kBlackboardGoalKey, current_goal);
        GetInputOrBlackboard(*this, config(), "goals", kBlackboardGoalsKey,
                     current_goals);
        if (!PoseStampedEqual(goal_, current_goal) ||
            !GoalsEqual(goals_, current_goals)) {
            goal_ = current_goal;
            goals_ = current_goals;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

private:
    commsgs::geometry_msgs::PoseStamped goal_;
    std::vector<commsgs::geometry_msgs::PoseStamped> goals_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(GoalUpdatedCondition, "GoalUpdated")

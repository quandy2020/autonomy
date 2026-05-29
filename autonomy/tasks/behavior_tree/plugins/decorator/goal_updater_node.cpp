/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/decorator_node.h"
namespace autonomy {
namespace tasks {
namespace behavior_tree {

class GoalUpdater : public BT::DecoratorNode
{
public:
    GoalUpdater(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::DecoratorNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<commsgs::geometry_msgs::PoseStamped>("input_goal"),
            BT::InputPort<std::vector<commsgs::geometry_msgs::PoseStamped>>(
                "input_goals"),
            BT::OutputPort<commsgs::geometry_msgs::PoseStamped>("output_goal"),
            BT::OutputPort<std::vector<commsgs::geometry_msgs::PoseStamped>>(
                "output_goals"),
        };
    }

    BT::NodeStatus tick() override {
        commsgs::geometry_msgs::PoseStamped goal;
        std::vector<commsgs::geometry_msgs::PoseStamped> goals;
        GetInputOrBlackboard(*this, config(), "input_goal", kBlackboardGoalKey, goal);
        GetInputOrBlackboard(*this, config(), "input_goals", kBlackboardGoalsKey, goals);
        setOutput("output_goal", goal);
        setOutput("output_goals", goals);
        config().blackboard->set(kBlackboardGoalKey, goal);
        config().blackboard->set(kBlackboardGoalsKey, goals);
        return child_node_->executeTick();
    }

};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(GoalUpdater, "GoalUpdater")

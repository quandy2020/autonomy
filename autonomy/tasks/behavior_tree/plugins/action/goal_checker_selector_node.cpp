/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class GoalCheckerSelector : public BT::SyncActionNode
{
public:
    GoalCheckerSelector(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::OutputPort<std::string>("selected_goal_checker"),
            BT::InputPort<std::string>("default_goal_checker", "", "Default"),
            BT::InputPort<std::string>("topic_name", "", "Unused"),
        };
    }

    BT::NodeStatus tick() override {
        std::string selected;
        std::string default_checker;
        getInput("default_goal_checker", default_checker);
        if (!config().blackboard->get("selected_goal_checker", selected) ||
            selected.empty()) {
            selected = default_checker;
        }
        if (selected.empty()) {
            config().blackboard->get("default_goal_checker_id", selected);
        }
        setOutput("selected_goal_checker", selected);
        config().blackboard->set("selected_goal_checker", selected);
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

REGISTER_BEHAVIOR_TREE_NODE(GoalCheckerSelector, "GoalCheckerSelector")

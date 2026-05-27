/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class PlannerSelectorNode : public BT::SyncActionNode
{
public:
    PlannerSelectorNode(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::OutputPort<std::string>("selected_planner"),
            BT::InputPort<std::string>("default_planner", "", "Default"),
            BT::InputPort<std::string>("topic_name", "", "Unused"),
        };
    }

    BT::NodeStatus tick() override {
        std::string selected;
        std::string default_planner;
        getInput("default_planner", default_planner);
        if (!config().blackboard->get("selected_planner", selected) ||
            selected.empty()) {
            selected = default_planner;
        }
        if (selected.empty()) {
            config().blackboard->get("default_planner_id", selected);
        }
        setOutput("selected_planner", selected);
        config().blackboard->set("selected_planner", selected);
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::PlannerSelectorNode>(
        "PlannerSelector");
}

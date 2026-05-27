/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class ControllerSelectorNode : public BT::SyncActionNode
{
public:
    ControllerSelectorNode(const std::string& name,
                           const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::OutputPort<std::string>("selected_controller"),
            BT::InputPort<std::string>("default_controller", "", "Default"),
            BT::InputPort<std::string>("topic_name", "", "Unused"),
        };
    }

    BT::NodeStatus tick() override {
        std::string selected;
        std::string default_controller;
        getInput("default_controller", default_controller);
        if (!config().blackboard->get("selected_controller", selected) ||
            selected.empty()) {
            selected = default_controller;
        }
        if (selected.empty()) {
            config().blackboard->get("default_controller_id", selected);
        }
        setOutput("selected_controller", selected);
        config().blackboard->set("selected_controller", selected);
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<
        autonomy::tasks::behavior_tree::ControllerSelectorNode>(
        "ControllerSelector");
}

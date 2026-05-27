/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

class SmootherSelectorNode : public BT::SyncActionNode
{
public:
    SmootherSelectorNode(const std::string& name,
                         const BT::NodeConfiguration& conf)
        : BT::SyncActionNode(name, conf) {}

    static BT::PortsList providedPorts() {
        return {
            BT::OutputPort<std::string>("selected_smoother"),
            BT::InputPort<std::string>("default_smoother", "", "Default"),
            BT::InputPort<std::string>("topic_name", "", "Unused"),
        };
    }

    BT::NodeStatus tick() override {
        std::string selected;
        std::string default_smoother;
        getInput("default_smoother", default_smoother);
        if (!config().blackboard->get("selected_smoother", selected) ||
            selected.empty()) {
            selected = default_smoother;
        }
        if (selected.empty()) {
            selected = "simple_smoother";
        }
        setOutput("selected_smoother", selected);
        config().blackboard->set("selected_smoother", selected);
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::SmootherSelectorNode>(
        "SmootherSelector");
}

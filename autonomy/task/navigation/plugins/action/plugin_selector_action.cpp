/*
 * Copyright 2026 The Openbot Authors
 *
 * Nav2-style Planner/Controller/Smoother selectors: copy default_* into
 * selected_* (topic subscriptions omitted; defaults from blackboard/ports).
 */

#include <string>

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"

namespace autonomy::task::plugins::navigation {

namespace {

BT::PortsList SelectorPorts(const char* selected, const char* default_port,
                            const char* topic)
{
    return {
        BT::OutputPort<std::string>(selected),
        BT::InputPort<std::string>(default_port),
        BT::InputPort<std::string>("topic_name", topic,
                                   "unused selector topic"),
    };
}

}  // namespace

class PlannerSelector : public BtSyncAction
{
public:
    PlannerSelector(const std::string& name, const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

    static BT::PortsList providedPorts()
    {
        return SelectorPorts("selected_planner", "default_planner",
                             "planner_selector");
    }

protected:
    BT::NodeStatus OnExecute() override
    {
        std::string value;
        getInput("default_planner", value);
        if (value.empty() && config().blackboard) {
            (void)config().blackboard->get("default_planner_id", value);
        }
        if (value.empty()) {
            return BT::NodeStatus::FAILURE;
        }
        setOutput("selected_planner", value);
        return BT::NodeStatus::SUCCESS;
    }
};

class ControllerSelector : public BtSyncAction
{
public:
    ControllerSelector(const std::string& name, const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

    static BT::PortsList providedPorts()
    {
        return SelectorPorts("selected_controller", "default_controller",
                             "controller_selector");
    }

protected:
    BT::NodeStatus OnExecute() override
    {
        std::string value;
        getInput("default_controller", value);
        if (value.empty() && config().blackboard) {
            (void)config().blackboard->get("default_controller_id", value);
        }
        if (value.empty()) {
            return BT::NodeStatus::FAILURE;
        }
        setOutput("selected_controller", value);
        return BT::NodeStatus::SUCCESS;
    }
};

class SmootherSelector : public BtSyncAction
{
public:
    SmootherSelector(const std::string& name, const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

    static BT::PortsList providedPorts()
    {
        return SelectorPorts("selected_smoother", "default_smoother",
                             "smoother_selector");
    }

protected:
    BT::NodeStatus OnExecute() override
    {
        std::string value;
        getInput("default_smoother", value);
        if (value.empty() && config().blackboard) {
            (void)config().blackboard->get("default_smoother_id", value);
        }
        if (value.empty()) {
            value = "simple_smoother";
        }
        setOutput("selected_smoother", value);
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::navigation::PlannerSelector>("PlannerSelector");
    factory.registerNodeType<
        autonomy::task::plugins::navigation::ControllerSelector>(
        "ControllerSelector");
    factory.registerNodeType<
        autonomy::task::plugins::navigation::SmootherSelector>(
        "SmootherSelector");
}

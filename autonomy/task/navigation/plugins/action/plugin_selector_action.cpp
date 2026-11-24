/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file plugin_selector_action.cpp
 * @brief Plugin selector action.
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

/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "autonomy/common/log.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "autonomy/tasks/behavior_tree/json_utils.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

using namespace std::chrono_literals;  // NOLINT

template <class ActionT>
class BtCancelActionNode : public BT::ActionNodeBase
{
public:
    BtCancelActionNode(const std::string& xml_tag_name,
                       const std::string& action_name,
                       const BT::NodeConfiguration& conf)
        : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name) {
        if (!GetInputPortOrBlackboard(*this, *config().blackboard,
                                      "server_timeout", server_timeout_)) {
            server_timeout_ = std::chrono::milliseconds(10);
        }
        wait_for_service_timeout_ =
            config().blackboard->template get<std::chrono::milliseconds>(
                "wait_for_service_timeout");

        std::string remapped_action_name;
        if (getInput("server_name", remapped_action_name)) {
            action_name_ = remapped_action_name;
        }
        ADEBUG << xml_tag_name.c_str() << " BtCancelActionNode initialized";
    }

    BtCancelActionNode() = delete;

    virtual ~BtCancelActionNode() = default;

    static BT::PortsList providedBasicPorts(BT::PortsList addition) {
        BT::PortsList basic = {
            BT::InputPort<std::string>("server_name", "Action name"),
            BT::InputPort<std::chrono::milliseconds>("server_timeout")};
        basic.insert(addition.begin(), addition.end());
        return basic;
    }

    void halt() override {}

    static BT::PortsList providedPorts() {
        return providedBasicPorts({});
    }

    BT::NodeStatus tick() override {
        setStatus(BT::NodeStatus::RUNNING);
        ADEBUG << "Cancel requested for action " << action_name_;
        return BT::NodeStatus::SUCCESS;
    }

protected:
    std::string action_name_;
    std::chrono::milliseconds server_timeout_;
    std::chrono::milliseconds wait_for_service_timeout_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

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

#include <string>

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/** In-process costmap clear (no service stub). */
class BtCostmapClearNode : public BtStatefulActionNode
{
public:
    BtCostmapClearNode(const std::string& xml_tag_name,
                       const BT::NodeConfiguration& conf)
        : BtStatefulActionNode(xml_tag_name, conf) {}

    static BT::PortsList providedBasicPorts(BT::PortsList addition) {
        BT::PortsList basic = {BT::InputPort<std::string>(
            "service_name", "please_set_service_name_in_BT_Node")};
        basic.insert(addition.begin(), addition.end());
        return basic;
    }

protected:
    std::string serviceNameFromPorts() const;

    BT::NodeStatus onRunning() override { return BT::NodeStatus::SUCCESS; }
};

class ClearEntireCostmapService : public BtCostmapClearNode
{
public:
    ClearEntireCostmapService(const std::string& service_node_name,
                              const BT::NodeConfiguration& conf);

    static BT::PortsList providedPorts() { return providedBasicPorts({}); }

    BT::NodeStatus onStart() override;
};

class ClearCostmapExceptRegionService : public BtCostmapClearNode
{
public:
    ClearCostmapExceptRegionService(const std::string& service_node_name,
                                    const BT::NodeConfiguration& conf);

    static BT::PortsList providedPorts() {
        return providedBasicPorts({BT::InputPort<double>(
            "reset_distance", 1,
            "Distance from the robot above which obstacles are cleared")});
    }

    BT::NodeStatus onStart() override;
};

class ClearCostmapAroundRobotService : public BtCostmapClearNode
{
public:
    ClearCostmapAroundRobotService(const std::string& service_node_name,
                                   const BT::NodeConfiguration& conf);

    static BT::PortsList providedPorts() {
        return providedBasicPorts({BT::InputPort<double>(
            "reset_distance", 1,
            "Distance from the robot under which obstacles are cleared")});
    }

    BT::NodeStatus onStart() override;
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

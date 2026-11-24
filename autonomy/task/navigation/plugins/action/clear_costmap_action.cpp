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
 * @file clear_costmap_action.cpp
 * @brief Clear costmap action.
 */

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/navigation/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::navigation {

class ClearCostmapAction : public BtSyncAction
{
public:
    ClearCostmapAction(const std::string& name, const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>(
                "service_name",
                "/global_costmap/clear_entirely_global_costmap",
                "clear-costmap service (currently fixed on client)"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }

protected:
    BT::NodeStatus OnExecute() override
    {
        // service_name is declared for BT XML compatibility; NavigationClient
        // already targets kClearGlobalCostmapService.
        std::string unused_service;
        getInput("service_name", unused_service);
        if (!ResolveClient(*this)->ClearCostmap()) {
            SetErrorPorts(*this, 1, "ClearCostmap: RPC failed");
            return BT::NodeStatus::FAILURE;
        }
        ClearErrorPorts(*this);
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::navigation::ClearCostmapAction>("ClearCostmap");
    factory.registerNodeType<
        autonomy::task::plugins::navigation::ClearCostmapAction>(
        "ClearEntireCostmap");
}

/*
 * Copyright 2026 The Openbot Authors
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
                "global_costmap/clear_entirely_global_costmap",
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
}

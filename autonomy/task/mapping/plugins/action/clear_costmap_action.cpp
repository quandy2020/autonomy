/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/mapping/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::mapping {

class ClearCostmapAction : public BtSyncAction
{
public:
    ClearCostmapAction(const std::string& name, const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

protected:
    BT::NodeStatus OnExecute() override
    {
        auto client = ResolveClient(*this);
        return (client && client->ClearCostmap()) ? BT::NodeStatus::SUCCESS
                                                  : BT::NodeStatus::FAILURE;
    }
};

}  // namespace autonomy::task::plugins::mapping

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::mapping::ClearCostmapAction>(
        "ClearCostmap");
}

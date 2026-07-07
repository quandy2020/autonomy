/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/apps/mapping/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::mapping {

class LoadMapAction : public BtSyncAction
{
public:
    LoadMapAction(const std::string& name, const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("map_name")};
    }

protected:
    BT::NodeStatus OnExecute() override
    {
        auto client = ResolveClient(*this);
        return (client && client->LoadMap()) ? BT::NodeStatus::SUCCESS
                                             : BT::NodeStatus::FAILURE;
    }
};

}  // namespace autonomy::task::plugins::mapping

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::mapping::LoadMapAction>(
        "LoadMap");
}

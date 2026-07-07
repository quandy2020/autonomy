/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/apps/charging/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::charging {

class DockSearchAction : public BtSyncAction
{
public:
    DockSearchAction(const std::string& name, const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

protected:
    BT::NodeStatus OnExecute() override
    {
        auto client = ResolveClient(*this);
        return (client && client->RunDockSearch()) ? BT::NodeStatus::SUCCESS
                                                   : BT::NodeStatus::FAILURE;
    }
};

}  // namespace autonomy::task::plugins::charging

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::charging::DockSearchAction>(
        "DockSearch");
}

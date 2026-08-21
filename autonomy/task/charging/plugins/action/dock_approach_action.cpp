/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/charging/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::charging {

class DockApproachAction : public BtSyncAction
{
public:
    DockApproachAction(const std::string& name, const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

protected:
    BT::NodeStatus OnExecute() override
    {
        auto client = ResolveClient(*this);
        if (!client || !client->IsChargerVisible()) {
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace autonomy::task::plugins::charging

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::charging::DockApproachAction>(
        "DockApproach");
}

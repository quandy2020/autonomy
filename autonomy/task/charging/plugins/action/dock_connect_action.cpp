/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/charging/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::charging {

class DockConnectAction : public BtSyncAction
{
public:
    DockConnectAction(const std::string& name, const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<float>("battery_target_percent", 100.f, "percent")};
    }

protected:
    BT::NodeStatus OnExecute() override
    {
        auto client = ResolveClient(*this);
        return (client && client->MarkConnected()) ? BT::NodeStatus::SUCCESS
                                                   : BT::NodeStatus::FAILURE;
    }
};

}  // namespace autonomy::task::plugins::charging

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::charging::DockConnectAction>(
        "DockConnect");
}

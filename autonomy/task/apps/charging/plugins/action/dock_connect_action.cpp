/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/apps/charging/plugins/plugin_utils.hpp"

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

class ChargerVisibleCondition : public BtCondition
{
public:
    ChargerVisibleCondition(const std::string& name, const BT::NodeConfig& config)
        : BtCondition(name, config) {}

protected:
    BT::NodeStatus OnEvaluate() override
    {
        auto client = ResolveClient(*this);
        if (!client) {
            return BT::NodeStatus::FAILURE;
        }
        return client->IsChargerVisible() ? BT::NodeStatus::SUCCESS
                                          : BT::NodeStatus::FAILURE;
    }
};

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
    factory.registerNodeType<autonomy::task::plugins::charging::DockApproachAction>(
        "DockApproach");
    factory.registerNodeType<
        autonomy::task::plugins::charging::ChargerVisibleCondition>("ChargerVisible");
    factory.registerNodeType<autonomy::task::plugins::charging::DockConnectAction>(
        "DockConnect");
}

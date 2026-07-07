/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/apps/localization/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::localization {

class StartLocalizationAction : public BtStatefulAction
{
public:
    StartLocalizationAction(const std::string& name, const BT::NodeConfig& config)
        : BtStatefulAction(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<int>("algorithm")};
    }

protected:
    BT::NodeStatus OnFirstTick() override
    {
        auto client = ResolveClient(*this);
        if (!client || !client->StartLocalization()) {
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus OnExecute() override
    {
        auto client = ResolveClient(*this);
        if (!client || !client->IsRunning()) {
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void OnHalted() override
    {
        if (auto client = ResolveClient(*this)) {
            client->StopLocalization();
        }
    }
};

}  // namespace autonomy::task::plugins::localization

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::localization::StartLocalizationAction>(
        "StartLocalization");
}

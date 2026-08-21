/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/charging/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::charging {

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

}  // namespace autonomy::task::plugins::charging

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::charging::ChargerVisibleCondition>("ChargerVisible");
}

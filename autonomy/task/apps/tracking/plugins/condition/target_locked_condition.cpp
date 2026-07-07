/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/apps/tracking/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::tracking {

class TargetLockedCondition : public BtCondition
{
public:
    TargetLockedCondition(const std::string& name, const BT::NodeConfig& config)
        : BtCondition(name, config) {}

protected:
    BT::NodeStatus OnEvaluate() override
    {
        auto client = ResolveClient(*this);
        if (!client) {
            return BT::NodeStatus::FAILURE;
        }
        return client->IsTargetLocked() ? BT::NodeStatus::SUCCESS
                                        : BT::NodeStatus::FAILURE;
    }
};

}  // namespace autonomy::task::plugins::tracking

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::tracking::TargetLockedCondition>("TargetLocked");
}

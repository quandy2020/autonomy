/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/apps/navigation/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::navigation {

class PlanningReadyCondition : public BtCondition
{
public:
    PlanningReadyCondition(const std::string& name, const BT::NodeConfig& config)
        : BtCondition(name, config) {}

protected:
    BT::NodeStatus OnEvaluate() override
    {
        return ResolveClient(*this)->IsPlanningReady() ? BT::NodeStatus::SUCCESS
                                                       : BT::NodeStatus::FAILURE;
    }
};

class ServersReadyCondition : public BtCondition
{
public:
    ServersReadyCondition(const std::string& name, const BT::NodeConfig& config)
        : BtCondition(name, config) {}

protected:
    BT::NodeStatus OnEvaluate() override
    {
        auto client = ResolveClient(*this);
        return (client->IsPlanningReady() && client->IsControlReady())
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
    }
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::navigation::PlanningReadyCondition>(
        "NavInitialPoseReceived");
    factory.registerNodeType<
        autonomy::task::plugins::navigation::ServersReadyCondition>(
        "NavTfAvailable");
}

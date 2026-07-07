/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/behavior_tree/plugins/bt_node_base.hpp"

namespace autonomy::task::plugins::localization {

class StopLocalizationAction : public BtSyncAction
{
public:
    StopLocalizationAction(const std::string& name, const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

protected:
    BT::NodeStatus OnExecute() override { return BT::NodeStatus::SUCCESS; }
};

class LocalizationReadyCondition : public BtCondition
{
public:
    LocalizationReadyCondition(const std::string& name,
                               const BT::NodeConfig& config)
        : BtCondition(name, config) {}

protected:
    BT::NodeStatus OnEvaluate() override { return BT::NodeStatus::SUCCESS; }
};

}  // namespace autonomy::task::plugins::localization

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::localization::StopLocalizationAction>(
        "StopLocalization");
    factory.registerNodeType<
        autonomy::task::plugins::localization::LocalizationReadyCondition>(
        "LocalizationReady");
}

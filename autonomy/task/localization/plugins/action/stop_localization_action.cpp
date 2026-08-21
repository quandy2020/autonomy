/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"

namespace autonomy::task::plugins::localization {

class StopLocalizationAction : public BtSyncAction
{
public:
    StopLocalizationAction(const std::string& name, const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

protected:
    BT::NodeStatus OnExecute() override { return BT::NodeStatus::SUCCESS; }
};

}  // namespace autonomy::task::plugins::localization

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::localization::StopLocalizationAction>(
        "StopLocalization");
}

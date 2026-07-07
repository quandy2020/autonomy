/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/apps/navigation/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::navigation {

class ClearCostmapAction : public BtSyncAction
{
public:
    ClearCostmapAction(const std::string& name, const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }

protected:
    BT::NodeStatus OnExecute() override
    {
        if (!ResolveClient(*this)->ClearCostmap()) {
            SetErrorPorts(*this, 1, "ClearCostmap: RPC failed");
            return BT::NodeStatus::FAILURE;
        }
        ClearErrorPorts(*this);
        return BT::NodeStatus::SUCCESS;
    }
};

class CancelMotionAction : public BtSyncAction
{
public:
    CancelMotionAction(const std::string& name, const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

protected:
    BT::NodeStatus OnExecute() override
    {
        ResolveClient(*this)->CancelActiveMotion();
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::navigation::ClearCostmapAction>("NavClearCostmap");
    factory.registerNodeType<
        autonomy::task::plugins::navigation::CancelMotionAction>(
        "NavControllerCancel");
}

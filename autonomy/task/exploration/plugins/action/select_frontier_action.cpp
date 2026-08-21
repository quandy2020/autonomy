/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/exploration/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::exploration {

class SelectFrontierAction : public BtSyncAction
{
public:
    SelectFrontierAction(const std::string& name, const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<automsgs::msgs::geometry_msgs::PoseStamped>(
                "goal", "next frontier pose"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }

protected:
    BT::NodeStatus OnExecute() override
    {
        auto client = ResolveClient(*this);
        if (!client) {
            SetErrorPorts(*this, 1, "SelectFrontier: missing exploration client");
            return BT::NodeStatus::FAILURE;
        }

        automsgs::msgs::geometry_msgs::PoseStamped goal;
        if (!client->SelectNextFrontier(goal)) {
            SetErrorPorts(*this, 2, "SelectFrontier: no frontier available");
            return BT::NodeStatus::FAILURE;
        }

        setOutput("goal", goal);
        ClearErrorPorts(*this);
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace autonomy::task::plugins::exploration

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::exploration::SelectFrontierAction>(
        "SelectFrontier");
}

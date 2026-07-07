/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/apps/tracking/plugins/plugin_utils.hpp"

namespace autonomy::task::plugins::tracking {

class ComputeFollowGoalAction : public BtSyncAction
{
public:
    ComputeFollowGoalAction(const std::string& name, const BT::NodeConfig& config)
        : BtSyncAction(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<commsgs::geometry_msgs::PoseStamped>("goal")};
    }

protected:
    BT::NodeStatus OnExecute() override
    {
        auto client = ResolveClient(*this);
        if (!client) {
            return BT::NodeStatus::FAILURE;
        }

        commsgs::geometry_msgs::PoseStamped goal;
        if (!client->ComputeFollowGoal(goal)) {
            return BT::NodeStatus::FAILURE;
        }

        setOutput("goal", goal);
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace autonomy::task::plugins::tracking

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::tracking::ComputeFollowGoalAction>(
        "ComputeFollowGoal");
}

/*
 * Copyright 2026 The Openbot Authors
 */

#include <automsgs/actions/nav_actions.pb.h>
#include "autonomy/task/navigation/plugins/plugin_utils.hpp"
#include "autonomy/task/navigation/plugins/async_remote_action_node.hpp"

namespace autonomy::task::plugins::navigation {

namespace navigation_actions = automsgs::actions;

class SpinAction : public AsyncRemoteActionNode<navigation_actions::SpinAction>
{
public:
    SpinAction(const std::string& name, const BT::NodeConfig& config)
        : AsyncRemoteActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("spin_dist", 1.57, "rad"),
            BT::InputPort<double>("time_allowance", 10.0, "sec"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }

protected:
    Client& GetClient(::autonomy::task::navigation::NavigationClient& client) override
    {
        return client.spin_client();
    }

    bool BuildGoal(Goal& goal) override
    {
        double spin_dist = 1.57;
        double allowance = 10.0;
        getInput("spin_dist", spin_dist);
        getInput("time_allowance", allowance);
        goal.set_target_yaw(static_cast<float>(spin_dist));
        *goal.mutable_time_allowance() = ToProtoDuration(allowance);
        return true;
    }

    const char* ServerLabel() const override { return "spin"; }
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::navigation::SpinAction>(
        "NavSpin");
}

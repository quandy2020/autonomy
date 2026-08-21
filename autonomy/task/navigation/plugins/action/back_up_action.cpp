/*
 * Copyright 2026 The Openbot Authors
 */

#include <automsgs/actions/nav_actions.pb.h>
#include "autonomy/task/navigation/plugins/plugin_utils.hpp"
#include "autonomy/task/navigation/plugins/async_remote_action_node.hpp"

namespace autonomy::task::plugins::navigation {

namespace navigation_actions = automsgs::actions;

class BackUpAction : public AsyncRemoteActionNode<navigation_actions::BackUpAction>
{
public:
    BackUpAction(const std::string& name, const BT::NodeConfig& config)
        : AsyncRemoteActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("backup_dist", 0.30, "m"),
            BT::InputPort<double>("backup_speed", 0.10, "m/s"),
            BT::InputPort<double>("time_allowance", 10.0, "sec"),
            BT::InputPort<bool>("is_recovery", false, "recovery behavior flag"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }

protected:
    Client& GetClient(::autonomy::task::navigation::NavigationClient& client) override
    {
        return client.backup_client();
    }

    bool BuildGoal(Goal& goal) override
    {
        double dist = 0.30;
        double speed = 0.10;
        double allowance = 10.0;
        getInput("backup_dist", dist);
        getInput("backup_speed", speed);
        getInput("time_allowance", allowance);
        goal.mutable_target()->set_x(-dist);
        goal.set_speed(static_cast<float>(speed));
        *goal.mutable_time_allowance() = ToProtoDuration(allowance);
        return true;
    }

    const char* ServerLabel() const override { return "backup"; }
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::navigation::BackUpAction>(
        "BackUp");
}

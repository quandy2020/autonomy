/*
 * Copyright 2026 The Openbot Authors
 */

#include <automsgs/actions/nav_actions.pb.h>
#include "autonomy/task/navigation/plugins/plugin_utils.hpp"
#include "autonomy/task/navigation/plugins/async_remote_action_node.hpp"

namespace autonomy::task::plugins::navigation {

namespace navigation_actions = automsgs::actions;

class WaitAction : public AsyncRemoteActionNode<navigation_actions::WaitAction>
{
public:
    WaitAction(const std::string& name, const BT::NodeConfig& config)
        : AsyncRemoteActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("wait_duration", 1.0, "sec"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }

protected:
    Client& GetClient(::autonomy::task::navigation::NavigationClient& client) override
    {
        return client.wait_client();
    }

    bool BuildGoal(Goal& goal) override
    {
        double duration = 1.0;
        getInput("wait_duration", duration);
        *goal.mutable_time() = ToProtoDuration(duration);
        return true;
    }

    const char* ServerLabel() const override { return "wait"; }
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::navigation::WaitAction>(
        "Wait");
}

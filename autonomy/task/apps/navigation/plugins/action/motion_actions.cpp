/*
 * Copyright 2026 The Openbot Authors
 */

#include <automsgs/actions/nav_actions.pb.h>
#include "autonomy/task/apps/navigation/plugins/plugin_utils.hpp"
#include "autonomy/task/apps/navigation/plugins/rpc_action_node.hpp"

namespace autonomy::task::plugins::navigation {

namespace nav_proto = automsgs::actions;

class SpinAction : public RpcAsyncActionNode<nav_proto::SpinAction>
{
public:
    SpinAction(const std::string& name, const BT::NodeConfig& config)
        : RpcAsyncActionNode(name, config) {}

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

class BackUpAction : public RpcAsyncActionNode<nav_proto::BackUpAction>
{
public:
    BackUpAction(const std::string& name, const BT::NodeConfig& config)
        : RpcAsyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("backup_dist", 0.30, "m"),
            BT::InputPort<double>("backup_speed", 0.10, "m/s"),
            BT::InputPort<double>("time_allowance", 10.0, "sec"),
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

class WaitAction : public RpcAsyncActionNode<nav_proto::WaitAction>
{
public:
    WaitAction(const std::string& name, const BT::NodeConfig& config)
        : RpcAsyncActionNode(name, config) {}

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
    factory.registerNodeType<autonomy::task::plugins::navigation::SpinAction>(
        "NavSpin");
    factory.registerNodeType<autonomy::task::plugins::navigation::BackUpAction>(
        "NavBackUp");
    factory.registerNodeType<autonomy::task::plugins::navigation::WaitAction>(
        "NavWait");
}

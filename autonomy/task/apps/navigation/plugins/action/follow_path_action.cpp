/*
 * Copyright 2026 The Openbot Authors
 */

#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/actions/nav_actions.pb.h>
#include "autonomy/task/apps/navigation/plugins/plugin_utils.hpp"
#include "autonomy/task/apps/navigation/plugins/rpc_action_node.hpp"

namespace autonomy::task::plugins::navigation {

namespace nav_proto = automsgs::actions;

class FollowPathAction : public RpcAsyncActionNode<nav_proto::FollowPathAction>
{
public:
    FollowPathAction(const std::string& name, const BT::NodeConfig& config)
        : RpcAsyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<automsgs::msgs::nav_msgs::Path>("path"),
            BT::InputPort<std::string>("controller_id"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }

protected:
    Client& GetClient(::autonomy::task::navigation::NavigationClient& client) override
    {
        return client.follow_path_client();
    }

    ::autonomy::task::navigation::ActionSession<nav_proto::FollowPathAction>&
    GetSession(::autonomy::task::navigation::NavigationClient& client) override
    {
        return client.follow_session();
    }

    bool BuildGoal(Goal& goal) override
    {
        automsgs::msgs::nav_msgs::Path path;
        std::string controller_id;
        if (!getInput("path", path) || path.poses().empty()) {
            SetErrorPorts(*this, 1, "FollowPath: missing path");
            return false;
        }
        getInput("controller_id", controller_id);

        *goal.mutable_path() = path;
        if (!controller_id.empty()) {
            goal.set_controller_id(controller_id);
        }
        return true;
    }

    const char* ServerLabel() const override { return "follow_path"; }
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<autonomy::task::plugins::navigation::FollowPathAction>(
        "NavFollowPath");
}

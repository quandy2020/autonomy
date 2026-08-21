/*
 * Copyright 2026 The Openbot Authors
 */

#include <cmath>
#include <string>

#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/actions/nav_actions.pb.h>
#include "autonomy/task/navigation/plugins/plugin_utils.hpp"
#include "autonomy/task/navigation/plugins/async_remote_action_node.hpp"

namespace autonomy::task::plugins::navigation {

namespace navigation_actions = automsgs::actions;

class FollowPathAction : public AsyncRemoteActionNode<navigation_actions::FollowPathAction>
{
public:
    FollowPathAction(const std::string& name, const BT::NodeConfig& config)
        : AsyncRemoteActionNode(name, config) {}

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

    ::autonomy::task::navigation::ActionSession<navigation_actions::FollowPathAction>&
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

        // Keep FollowPath RPC small: a dense global path (200+ poses) can exceed
        // the default 16KB SHM slot and leave send_goal without a response.
        constexpr int kMaxPoses = 40;
        if (path.poses_size() > kMaxPoses) {
            automsgs::msgs::nav_msgs::Path sparse;
            sparse.mutable_header()->CopyFrom(path.header());
            const int n = path.poses_size();
            *sparse.add_poses() = path.poses(0);
            for (int i = 1; i < kMaxPoses - 1; ++i) {
                const int idx = (i * (n - 1)) / (kMaxPoses - 1);
                *sparse.add_poses() = path.poses(idx);
            }
            *sparse.add_poses() = path.poses(n - 1);
            *goal.mutable_path() = sparse;
        } else {
            *goal.mutable_path() = path;
        }
        if (!controller_id.empty()) {
            goal.set_controller_id(controller_id);
        }
        return true;
    }

    const char* ServerLabel() const override { return "follow_path"; }

private:
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::navigation::FollowPathAction>("FollowPath");
}

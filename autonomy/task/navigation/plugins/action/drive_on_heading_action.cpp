/*
 * Copyright 2026 The Openbot Authors
 *
 * Forward drive via the backup action server (positive target.x).
 */

#include <automsgs/actions/nav_actions.pb.h>
#include "autonomy/task/navigation/plugins/plugin_utils.hpp"
#include "autonomy/task/navigation/plugins/async_remote_action_node.hpp"

namespace autonomy::task::plugins::navigation {

namespace navigation_actions = automsgs::actions;

class DriveOnHeadingAction
    : public AsyncRemoteActionNode<navigation_actions::BackUpAction>
{
public:
    DriveOnHeadingAction(const std::string& name, const BT::NodeConfig& config)
        : AsyncRemoteActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("dist_to_travel", 0.15, "m"),
            BT::InputPort<double>("speed", 0.025, "m/s"),
            BT::InputPort<double>("time_allowance", 10.0, "sec"),
            BT::InputPort<bool>("disable_collision_checks", false, ""),
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
        double dist = 0.15;
        double speed = 0.025;
        double allowance = 10.0;
        getInput("dist_to_travel", dist);
        getInput("speed", speed);
        getInput("time_allowance", allowance);
        goal.mutable_target()->set_x(dist);
        goal.set_speed(static_cast<float>(speed));
        *goal.mutable_time_allowance() = ToProtoDuration(allowance);
        return true;
    }

    const char* ServerLabel() const override { return "drive_on_heading"; }
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::navigation::DriveOnHeadingAction>(
        "DriveOnHeading");
}

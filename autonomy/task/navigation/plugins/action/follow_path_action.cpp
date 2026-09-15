/*
 * Copyright 2026 The Openbot Authors
 */

#include <cmath>
#include <cstdint>
#include <string>

#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/actions/nav_actions.pb.h>
#include "autonomy/common/logging.hpp"
#include "autonomy/task/navigation/plugins/plugin_utils.hpp"
#include "autonomy/task/navigation/plugins/async_remote_action_node.hpp"

namespace autonomy::task::plugins::navigation {

namespace navigation_actions = automsgs::actions;

namespace {

uint64_t PathFingerprint(const automsgs::msgs::nav_msgs::Path& path)
{
    const int n = path.poses_size();
    if (n <= 0) {
        return 0;
    }
    // Fingerprint goal endpoint only (5 cm). Replans from a moving start pose
    // change path length/start every tick; those must not force FollowPath
    // replace when the navigation goal is unchanged.
    const auto& b = path.poses(n - 1).pose().position();
    const auto q = [](double v) {
        return static_cast<uint64_t>(std::llround(v * 20.0)) & 0xfffffu;
    };
    return (q(b.x()) << 20) ^ q(b.y());
}

}  // namespace

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
            AWARN_EVERY(20) << "FollowPath: missing path on blackboard";
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
        pending_fingerprint_ = PathFingerprint(goal.path());
        return true;
    }

    BT::NodeStatus OnFirstTick() override
    {
        auto client = ResolveClient(*this);
        if (!GetClient(*client).ActionServerIsReady()) {
            AWARN_EVERY(50) << "follow_path action server not ready; waiting";
            remote_started_ = false;
            return BT::NodeStatus::RUNNING;
        }

        Goal goal;
        if (!BuildGoal(goal)) {
            remote_started_ = false;
            return BT::NodeStatus::FAILURE;
        }

        auto& session = GetSession(*client);
        // RateController replans often publish a near-identical /plan. Replacing
        // an in-flight FollowPath just spams preempt + unknown-goal feedback.
        if (session.is_busy() && pending_fingerprint_ != 0 &&
            pending_fingerprint_ == sent_fingerprint_) {
            remote_started_ = true;
            return BT::NodeStatus::RUNNING;
        }

        AINFO_EVERY(10) << "FollowPath: sending path (" << goal.path().poses_size()
                        << " poses) controller_id=" << goal.controller_id();
        session.Begin(GetClient(*client), goal);
        sent_fingerprint_ = pending_fingerprint_;
        remote_started_ = true;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus OnExecute() override
    {
        // If blackboard path changed materially while following, upgrade the
        // active goal once instead of waiting for a node halt/restart.
        automsgs::msgs::nav_msgs::Path path;
        if (getInput("path", path) && !path.poses().empty()) {
            Goal goal;
            if (BuildGoal(goal) && pending_fingerprint_ != 0 &&
                pending_fingerprint_ != sent_fingerprint_) {
                auto client = ResolveClient(*this);
                AINFO_EVERY(10) << "FollowPath: path updated ("
                                << goal.path().poses_size() << " poses)";
                GetSession(*client).Begin(GetClient(*client), goal);
                sent_fingerprint_ = pending_fingerprint_;
            }
        }
        return AsyncRemoteActionNode::OnExecute();
    }

    void OnHalted() override
    {
        AsyncRemoteActionNode::OnHalted();
        sent_fingerprint_ = 0;
        pending_fingerprint_ = 0;
    }

    const char* ServerLabel() const override { return "follow_path"; }

private:
    uint64_t sent_fingerprint_{0};
    uint64_t pending_fingerprint_{0};
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::navigation::FollowPathAction>("FollowPath");
}

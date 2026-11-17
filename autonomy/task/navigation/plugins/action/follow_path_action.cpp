/*
 * Copyright 2026 The Openbot Authors
 */

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/actions/nav_actions.pb.h>
#include "autonomy/common/logging.hpp"
#include "autonomy/task/navigation/plugins/plugin_utils.hpp"
#include "autonomy/task/navigation/plugins/async_remote_action_node.hpp"
#include "autonomy/task/navigation/path_update.hpp"

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
            BT::InputPort<uint64_t>("path_revision", 0, "installed path version"),
            BT::InputPort<double>("goal_reached_tol", 0.10, "m"),
            BT::InputPort<double>("goal_reached_yaw_tol", 0.10, "rad"),
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
            AWARN_EVERY(20) << "nav follow missing_path";
            SetErrorPorts(*this, 1, "FollowPath: missing path");
            return false;
        }
        getInput("controller_id", controller_id);

        // Keep FollowPath RPC within the SHM slot, but do not stride a long
        // NavFn path down to 40 points — that makes ~0.5 m kinks and MPPI
        // hunts (vx sawtooth on long corridors).
        constexpr int kMaxPoses = 100;
        constexpr double kMinSpacing = 0.15;
        if (path.poses_size() > kMaxPoses) {
            automsgs::msgs::nav_msgs::Path sparse;
            sparse.mutable_header()->CopyFrom(path.header());
            const int n = path.poses_size();
            *sparse.add_poses() = path.poses(0);
            double acc = 0.0;
            for (int i = 1; i < n - 1; ++i) {
                const auto& a = path.poses(i - 1).pose().position();
                const auto& b = path.poses(i).pose().position();
                acc += std::hypot(b.x() - a.x(), b.y() - a.y());
                if (acc < kMinSpacing) {
                    continue;
                }
                acc = 0.0;
                *sparse.add_poses() = path.poses(i);
            }
            *sparse.add_poses() = path.poses(n - 1);
            if (sparse.poses_size() > kMaxPoses) {
                automsgs::msgs::nav_msgs::Path capped;
                capped.mutable_header()->CopyFrom(sparse.header());
                const int m = sparse.poses_size();
                *capped.add_poses() = sparse.poses(0);
                for (int i = 1; i < kMaxPoses - 1; ++i) {
                    *capped.add_poses() =
                        sparse.poses((i * (m - 1)) / (kMaxPoses - 1));
                }
                *capped.add_poses() = sparse.poses(m - 1);
                *goal.mutable_path() = capped;
            } else {
                *goal.mutable_path() = sparse;
            }
        } else {
            *goal.mutable_path() = path;
        }
        if (!controller_id.empty()) {
            goal.set_controller_id(controller_id);
        }
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
            GetSession(*client).Cancel(GetClient(*client));
            has_sent_goal_ = false;
            return BT::NodeStatus::FAILURE;
        }

        auto& session = GetSession(*client);
        // RateController replans often publish a near-identical /plan. Replacing
        // an in-flight FollowPath just spams preempt + unknown-goal feedback.
        if (session.is_busy() && !NeedsReplacement(goal)) {
            remote_started_ = true;
            return BT::NodeStatus::RUNNING;
        }

        AINFO << "nav follow send poses=" << goal.path().poses_size()
              << " controller=" << goal.controller_id();
        session.Begin(GetClient(*client), goal);
        RememberGoal(goal);
        remote_started_ = true;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus OnExecute() override
    {
        // If blackboard path changed materially while following, upgrade the
        // active goal once instead of waiting for a node halt/restart.
        automsgs::msgs::nav_msgs::Path path;
        if (!getInput("path", path) || path.poses().empty()) {
            auto client = ResolveClient(*this);
            GetSession(*client).Cancel(GetClient(*client));
            has_sent_goal_ = false;
            remote_started_ = false;
            return BT::NodeStatus::RUNNING;
        }
        if (getInput("path", path) && !path.poses().empty()) {
            Goal goal;
            if (BuildGoal(goal) && NeedsReplacement(goal)) {
                auto client = ResolveClient(*this);
                AINFO << "nav follow update poses=" << goal.path().poses_size();
                GetSession(*client).Begin(GetClient(*client), goal);
                RememberGoal(goal);
                remote_started_ = true;
            }
        }
        const auto status = AsyncRemoteActionNode::OnExecute();
        if (status == BT::NodeStatus::SUCCESS && has_sent_goal_) {
            double xy = 0.10, yaw = 0.10;
            getInput("goal_reached_tol", xy);
            getInput("goal_reached_yaw_tol", yaw);
            const auto& endpoint = sent_goal_.path().poses(sent_goal_.path().poses_size()-1);
            if (!ResolveClient(*this)->IsGoalReached(endpoint, xy, yaw)) {
                SetErrorPorts(*this, 1, "FollowPath success without fresh full-pose arrival");
                return BT::NodeStatus::FAILURE;
            }
        }
        return status;
    }

    void OnHalted() override
    {
        AsyncRemoteActionNode::OnHalted();
        has_sent_goal_ = false;
    }

    const char* ServerLabel() const override { return "follow_path"; }

private:
    int GoalEpoch() const
    {
        int epoch = 0;
        if (config().blackboard) config().blackboard->get("goal_epoch", epoch);
        return epoch;
    }
    bool NeedsReplacement(const Goal& goal) const
    {
        // Ignore path_revision: ComputePath bumps it on every RateController
        // tick. Preempt+MPPI reset on a 4 s replan is what stalls the robot.
        if (!has_sent_goal_ || GoalEpoch() != sent_epoch_) {
            return true;
        }
        if (goal.controller_id() != sent_goal_.controller_id()) {
            return true;
        }
        return !::autonomy::task::navigation::EquivalentPathSuffix(
            goal.path(), sent_goal_.path(), 0.35);
    }
    void RememberGoal(const Goal& goal)
    {
        sent_goal_ = goal;
        sent_epoch_ = GoalEpoch();
        getInput("path_revision", sent_path_revision_);
        has_sent_goal_ = true;
    }
    Goal sent_goal_;
    int sent_epoch_{0};
    bool has_sent_goal_{false};
    uint64_t sent_path_revision_{0};
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::navigation::FollowPathAction>("FollowPath");
}

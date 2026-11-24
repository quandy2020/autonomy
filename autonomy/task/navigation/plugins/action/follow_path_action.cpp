/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file follow_path_action.cpp
 * @brief Follow current paths through revocable, invocation-owned dispatch.
 */

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/actions/nav_actions.pb.h>
#include "autonomy/common/logging.hpp"
#include "autonomy/task/navigation/plugins/plugin_utils.hpp"
#include "autonomy/task/navigation/plugins/async_remote_action_node.hpp"
#include "autonomy/task/navigation/path_reference.hpp"

namespace autonomy::task::plugins::navigation {

namespace navigation_actions = automsgs::actions;

/**
 * @brief Follow complete route geometry with revocable
          * replacement/resumption.
 */
class FollowPathAction : public AsyncRemoteActionNode<navigation_actions::FollowPathAction>
{
public:
    /**
     * @brief Bind the native BT name and its shared navigation context.
     */
    FollowPathAction(const std::string& name, const BT::NodeConfig& config)
        : AsyncRemoteActionNode(name, config) {}

    /**
     * @brief Expose route, controller, transport budget and error ports.
     */
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<automsgs::msgs::nav_msgs::Path>("path"),
            BT::InputPort<std::string>("controller_id"),
            BT::InputPort<int>("max_path_poses", 40,
                               "Configured transport pose budget; necessary bends are never removed"),
            BT::InputPort<std::uint64_t>(
                "path_revision", 0,
                "Installed path version from ComputePathToPose; retained for XML compatibility"),
            BT::OutputPort<int>("error_code_id"),
            BT::OutputPort<std::string>("error_msg"),
        };
    }

protected:
    /**
     * @brief Resolve the native FollowPath RPC client owned by navigation.
     */
    Client& GetClient(::autonomy::task::navigation::NavigationClient& client) override
    {
        return client.follow_path_client();
    }

    /**
     * @brief Reuse this navigation invocation's retained FollowPath session.
     */
    ::autonomy::task::navigation::ActionSession<navigation_actions::FollowPathAction>&
    GetSession(::autonomy::task::navigation::NavigationClient& client) override
    {
        return client.follow_session();
    }

    /**
     * @brief Validate and compact a complete route without deleting bends.
     */
    bool BuildGoal(Goal& goal) override
    {
        automsgs::msgs::nav_msgs::Path path;
        std::string controller_id;
        arrived_without_path_ = false;
        pending_reference_.reset();
        if (!getInput("path", path) || path.poses().empty()) {
            std::vector<automsgs::msgs::geometry_msgs::PoseStamped> goals;
            if (config().blackboard &&
                config().blackboard->get("goals", goals) && goals.empty()) {
                ClearErrorPorts(*this);
                arrived_without_path_ = true;
                return false;
            }
            AWARN_EVERY(20) << "FollowPath: missing path on blackboard";
            SetErrorPorts(*this, 1, "FollowPath: missing path");
            return false;
        }
        getInput("controller_id", controller_id);
        try {
            std::vector<::autonomy::task::navigation::PathReferencePose> poses;
            poses.reserve(static_cast<std::size_t>(path.poses_size()));
            for (const auto& stamped : path.poses()) {
                if (!stamped.header().frame_id().empty() &&
                    stamped.header().frame_id() != path.header().frame_id())
                    throw std::invalid_argument("path pose uses a different coordinate frame");
                const auto& position = stamped.pose().position();
                const auto& rotation = stamped.pose().orientation();
                poses.push_back({{position.x(), position.y(), position.z()},
                                 {rotation.x(), rotation.y(), rotation.z(), rotation.w()}});
            }
            pending_reference_.emplace(path.header().frame_id(), controller_id, std::move(poses));
            int max_path_poses = 40;
            getInput("max_path_poses", max_path_poses);
            const auto& retained = pending_reference_->retained_indices();
            if (max_path_poses < 1 || retained.size() > static_cast<std::size_t>(max_path_poses))
                throw std::invalid_argument("complete path exceeds configured transport pose budget; "
                                            "necessary bends cannot be removed");
            // Only numerically redundant collinear poses are removed. Uniform
            // index sampling can shortcut a narrow doorway or obstacle detour.
            goal.mutable_path()->mutable_header()->CopyFrom(path.header());
            for (const auto index : retained)
                *goal.mutable_path()->add_poses() = path.poses(static_cast<int>(index));
            if (!controller_id.empty()) goal.set_controller_id(controller_id);
        } catch (const std::exception& error) {
            pending_reference_.reset();
            SetErrorPorts(*this, 1, std::string("FollowPath: ") + error.what());
            return false;
        }
        return true;
    }

    /**
     * @brief Admit a complete route or retain a concurrently paused session.
     */
    BT::NodeStatus OnFirstTick() override
    {
        auto client = ResolveClient(*this);
        if (!GetClient(*client).ActionServerIsReady()) {
            AWARN_EVERY(50) << "follow_path action server not ready; waiting";
            remote_started_ = false;
            return BT::NodeStatus::RUNNING;
        }

        auto& session = GetSession(*client);
        const auto gate = ResolveMotionGate(*this);
        Goal goal;
        if (!BuildGoal(goal)) {
            AWARN << "FollowPath: goal rebuild failed";
            if (gate && gate->Suspended()) {
                remote_started_ = session.is_busy();
                return BT::NodeStatus::RUNNING;
            }
            if (session.is_busy()) session.Cancel(GetClient(*client));
            remote_started_ = false;
            return arrived_without_path_ ? BT::NodeStatus::SUCCESS
                                         : BT::NodeStatus::FAILURE;
        }

        // RateController replans often publish a near-identical /plan. Replacing
        // an in-flight FollowPath just spams preempt + unknown-goal feedback.
        if (session.is_busy() && !session.NeedsResume(gate) &&
            sent_reference_ && pending_reference_ &&
            sent_reference_->HasSameRemainingPath(*pending_reference_)) {
            remote_started_ = true;
            return BT::NodeStatus::RUNNING;
        }

        AINFO_EVERY(10) << "FollowPath: sending path (" << goal.path().poses_size()
                        << " poses) controller_id=" << goal.controller_id();
        if (!session.Begin(GetClient(*client), goal, gate)) {
            // Pause may close admission after BuildGoal. Keep a retained action
            // latched so OnExecute can resume it after authoritative termination.
            if (gate && gate->Suspended()) {
                remote_started_ = session.is_busy();
                return BT::NodeStatus::RUNNING;
            }
            remote_started_ = false;
            SetErrorPorts(*this, -1, "follow_path dispatch admission revoked");
            return BT::NodeStatus::FAILURE;
        }
        OnGoalDispatched(goal);
        remote_started_ = true;
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Revalidate rolling references before polling or resuming
              * execution.
     */
    BT::NodeStatus OnExecute() override
    {
        auto client = ResolveClient(*this);
        auto& session = GetSession(*client);
        const auto gate = ResolveMotionGate(*this);
        // If blackboard path changed materially while following, upgrade the
        // active goal once instead of waiting for a node halt/restart.
        Goal goal;
        if (!BuildGoal(goal)) {
            if (gate && gate->Suspended()) return BT::NodeStatus::RUNNING;
            // A malformed or untransmittable new route cannot silently retain
            // a preceding route which may now cross an obstacle. An exhausted
            // goal list also completes a paused/resumable invocation here.
            session.Cancel(GetClient(*client));
            remote_started_ = false;
            return arrived_without_path_ ? BT::NodeStatus::SUCCESS
                                         : BT::NodeStatus::FAILURE;
        }

        if (session.NeedsResume(gate))
            return AsyncRemoteActionNode::OnExecute();

        if (!sent_reference_ ||
            !sent_reference_->HasSameRemainingPath(*pending_reference_)) {
            AINFO_EVERY(10) << "FollowPath: path updated ("
                            << goal.path().poses_size() << " poses)";
            if (!session.Begin(GetClient(*client), goal, gate)) {
                // Begin preserves the preceding session on refused admission;
                // a concurrent pause must not publish an unadmitted reference.
                if (gate && gate->Suspended()) return BT::NodeStatus::RUNNING;
                SetErrorPorts(*this, -1, "updated follow_path dispatch admission revoked");
                return BT::NodeStatus::FAILURE;
            }
            OnGoalDispatched(goal);
        }
        return AsyncRemoteActionNode::OnExecute();
    }

    /**
     * @brief Cancel the matching invocation and release its route references.
     */
    void OnHalted() override
    {
        AINFO << "FollowPath: halted by behavior tree";
        AsyncRemoteActionNode::OnHalted();
        sent_reference_.reset();
        pending_reference_.reset();
    }

    /**
     * @brief Identify the remote service in readiness and dispatch
              * diagnostics.
     */
    const char* ServerLabel() const override { return "follow_path"; }

    /**
     * @brief Follow the latest absolute path after a confirmed paused stop.
     * The controller resolves progress from current pose; rebuilding also
     * verifies that a usable path remains available on the blackboard.
     */
    bool BuildResumeGoal(Goal& goal) override {
        return BuildGoal(goal);
    }

    /**
     * @brief Commit initial, rolling and resumed bookkeeping after admission.
     */
    void OnGoalDispatched(const Goal& /*goal*/) noexcept override {
        if (pending_reference_) {
            sent_reference_.swap(pending_reference_);
            pending_reference_.reset();
        }
    }

private:
    using Reference = ::autonomy::task::navigation::PathReference;
    std::optional<Reference> sent_reference_;    ///< Complete locally admitted route.
    std::optional<Reference> pending_reference_; ///< Candidate awaiting local admission.
    bool arrived_without_path_{false}; ///< Empty goal list means navigation finished.
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::navigation::FollowPathAction>("FollowPath");
}

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
 * @file async_remote_action_node.hpp
 * @brief Native action polling with revocable dispatch and explicit resumption.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_PLUGINS_ASYNC_REMOTE_ACTION_NODE_HPP_
#define AUTONOMY_TASK_NAVIGATION_PLUGINS_ASYNC_REMOTE_ACTION_NODE_HPP_

#include <string>

#include "autonomy/common/logging.hpp"
#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/navigation/action_session.hpp"
#include "autonomy/task/navigation/plugins/plugin_utils.hpp"
#include "autonomy/task/common/task_action_client.hpp"

namespace autonomy::task::plugins::navigation {

/**
 * @brief Poll native actions through invocation-owned, revocable dispatch.
 * @tparam ActionT Native action with Goal, Feedback and Result messages.
 */
template <typename ActionT>
class AsyncRemoteActionNode : public BtStatefulAction
{
public:
    using Result = typename ActionT::Result; ///< Authoritative action payload.

    /**
     * @brief Bind the BT node name and shared navigation blackboard.
     */
    AsyncRemoteActionNode(const std::string& name, const BT::NodeConfig& config)
        : BtStatefulAction(name, config) {}

protected:
    using Goal = typename ActionT::Goal; ///< Request rebuilt by the concrete node.
    using Client = common::TaskActionClient<ActionT>; ///< Native task RPC facade.

    /**
     * @brief Resolves the specific RPC client from the navigation owner.
     * @param[in] client Navigation client that owns every action client.
     * @return Reference to the action client this node drives.
     */
    virtual Client& GetClient(::autonomy::task::navigation::NavigationClient& client) = 0;
    /**
     * @brief Returns the retained invocation session used for this action.
     * @param[in] client Navigation client; the default implementation ignores it
     *            and reuses the node-local session.
     * @return Session that owns the goal handle and its cancellation state.
     */
    virtual ::autonomy::task::navigation::ActionSession<ActionT>& GetSession(
        ::autonomy::task::navigation::NavigationClient& client)
    {
        (void)client;
        return local_session_;
    }
    /**
     * @brief Validates and populates an action request before local admission.
     * @param[out] goal Request filled from the node ports.
     * @return false when the ports do not describe a usable request.
     */
    virtual bool BuildGoal(Goal& goal) = 0;
    /**
     * @brief Rebuild retained intent after confirmed cancellation and resume.
     * @param goal Fresh request to execute in the resumed revision.
     * @return False unless the concrete action can preserve remaining intent.
     * Repeating the original relative Spin/BackUp/Wait would duplicate already
     * executed work. Such nodes need an explicit continuation implementation.
     */
    virtual bool BuildResumeGoal(Goal& /*goal*/) { return false; }
    /**
     * @brief Publish reference bookkeeping after local dispatch admission.
     * @param goal The newly admitted request, including a resumed request.
     * Begin success does not prove remote acceptance or physical movement. A
     * candidate built during a refused pause must not become admitted state.
     */
    virtual void OnGoalDispatched(const Goal& /*goal*/) noexcept {}
    /**
     * @brief Identify the action in readiness and failure diagnostics.
     */
    virtual const char* ServerLabel() const = 0;

    /**
     * @brief Latch execution only after the local session has been started.
     */
    bool LatchAfterFirstTick() const override { return remote_started_; }

    /**
     * @brief Check readiness and commit one invocation-owned request.
     */
    BT::NodeStatus OnFirstTick() override
    {
        auto client = ResolveClient(*this);
        if (!GetClient(*client).ActionServerIsReady()) {
            AWARN_EVERY(50) << ServerLabel()
                            << " action server not ready; waiting";
            remote_started_ = false;
            return BT::NodeStatus::RUNNING;
        }

        Goal goal;
        if (!BuildGoal(goal)) {
            remote_started_ = false;
            return BT::NodeStatus::FAILURE;
        }

        const auto gate = ResolveMotionGate(*this);
        if (!GetSession(*client).Begin(GetClient(*client), goal, gate)) {
            remote_started_ = false;
            if (gate && gate->Suspended()) return BT::NodeStatus::RUNNING;
            SetErrorPorts(*this, -1, "motion dispatch admission revoked");
            return BT::NodeStatus::FAILURE;
        }
        OnGoalDispatched(goal);
        remote_started_ = true;
        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief Resume confirmed remaining intent, or poll the admitted session.
     */
    BT::NodeStatus OnExecute() override
    {
        auto client = ResolveClient(*this);
        auto& session = GetSession(*client);
        const auto gate = ResolveMotionGate(*this);
        if (!isHaltRequested() && session.NeedsResume(gate)) {
            Goal goal;
            if (!BuildResumeGoal(goal)) {
                if (gate->Suspended()) return BT::NodeStatus::RUNNING;
                SetErrorPorts(*this, -1, "action resume lacks remaining execution intent");
                session.Reset();
                return BT::NodeStatus::FAILURE;
            }
            if (!GetClient(*client).ActionServerIsReady())
                return BT::NodeStatus::RUNNING;
            if (!session.Begin(GetClient(*client), goal, gate)) {
                if (gate->Suspended()) return BT::NodeStatus::RUNNING;
                SetErrorPorts(*this, -1, "resumed motion dispatch admission revoked");
                return BT::NodeStatus::FAILURE;
            }
            OnGoalDispatched(goal);
            return BT::NodeStatus::RUNNING;
        }
        const auto status = session.Tick(
            HaltCancelChecker(),
            [this](const Result& result) {
                setOutput("error_code_id", static_cast<int>(result.error_code()));
                setOutput("error_msg", result.error_msg());
            },
            [this](int code, const std::string& msg) {
                AWARN << ServerLabel() << " result failure code=" << code
                      << " msg=" << msg;
                SetErrorPorts(*this, code, msg);
            });
        if (status == BT::NodeStatus::SUCCESS)
            AINFO << ServerLabel() << " completed successfully";
        else if (status == BT::NodeStatus::FAILURE)
            AWARN << ServerLabel() << " session returned failure";
        return status;
    }

    /**
     * @brief Initiate goal-specific cancellation without fabricating a stop.
     */
    void OnHalted() override
    {
        auto client = ResolveClient(*this);
        GetSession(*client).Cancel(GetClient(*client));
        remote_started_ = false;
    }

protected:
    ::autonomy::task::navigation::ActionSession<ActionT> local_session_;
    ///< Retained local invocation when navigation does not supply a shared one.
    bool remote_started_{false}; ///< Controls the BT first-tick execution latch.
};

}  // namespace autonomy::task::plugins::navigation

#endif  // AUTONOMY_TASK_NAVIGATION_PLUGINS_ASYNC_REMOTE_ACTION_NODE_HPP_

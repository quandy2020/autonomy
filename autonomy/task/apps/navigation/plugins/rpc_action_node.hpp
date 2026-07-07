/*
 * Copyright 2026 The Openbot Authors
 *
 * Shared BT scaffolding for cross-process action RPC.
 */

#pragma once

#include <string>

#include "autonomy/task/apps/behavior_tree/plugins/bt_node_base.hpp"
#include "autonomy/task/apps/navigation/action_session.hpp"
#include "autonomy/task/apps/navigation/plugins/plugin_utils.hpp"
#include "autonomy/task/common/task_action_client.hpp"

namespace autonomy::task::plugins::navigation {

/** Async action node: send goal on first tick, poll until done. */
template <typename ActionT>
class RpcAsyncActionNode : public BtStatefulAction
{
public:
    using Result = typename ActionT::Result;

    RpcAsyncActionNode(const std::string& name, const BT::NodeConfig& config)
        : BtStatefulAction(name, config) {}

protected:
    using Goal = typename ActionT::Goal;
    using Client = common::TaskActionClient<ActionT>;

    virtual Client& GetClient(::autonomy::task::navigation::NavigationClient& client) = 0;
    virtual ::autonomy::task::navigation::ActionSession<ActionT>& GetSession(
        ::autonomy::task::navigation::NavigationClient& client)
    {
        (void)client;
        return local_session_;
    }
    virtual bool BuildGoal(Goal& goal) = 0;
    virtual const char* ServerLabel() const = 0;

    BT::NodeStatus OnFirstTick() override
    {
        auto client = ResolveClient(*this);
        if (!GetClient(*client).ActionServerIsReady()) {
            SetErrorPorts(*this, 1, std::string(ServerLabel()) + " not ready");
            return BT::NodeStatus::FAILURE;
        }

        Goal goal;
        if (!BuildGoal(goal)) {
            return BT::NodeStatus::FAILURE;
        }

        GetSession(*client).Begin(GetClient(*client), goal);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus OnExecute() override
    {
        auto client = ResolveClient(*this);
        return GetSession(*client).Tick(
            HaltCancelChecker(),
            [this](const Result& result) {
                setOutput("error_code_id", static_cast<int>(result.error_code()));
                setOutput("error_msg", result.error_msg());
            },
            [this](int code, const std::string& msg) {
                SetErrorPorts(*this, code, msg);
            });
    }

    void OnHalted() override
    {
        auto client = ResolveClient(*this);
        GetSession(*client).Cancel(GetClient(*client));
    }

protected:
    ::autonomy::task::navigation::ActionSession<ActionT> local_session_;

private:
};

}  // namespace autonomy::task::plugins::navigation

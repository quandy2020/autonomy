/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "autolink/autolink.hpp"
#include "autonomy/common/logging.hpp"
#include "autolink/action/create_client.hpp"
#include "autolink/action/types.hpp"
#include "autolink/node/node.hpp"
#include "autonomy/tasks/behavior_tree/bt_context.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/**
 * @brief BT leaf that calls an autolink action server (nav2 BtActionNode analogue).
 */
template <typename ActionT>
class BtActionNode : public BT::ActionNodeBase
{
public:
    using Goal = typename ActionT::Goal;
    using Feedback = typename ActionT::Feedback;
    using Result = typename ActionT::Result;
    using ActionClient = autolink::action::Client<ActionT>;
    using GoalHandle = autolink::action::ClientGoalHandle<ActionT>;
    using WrappedResult = typename GoalHandle::WrappedResult;

    BtActionNode(const std::string& xml_tag_name,
                           const std::string& action_name,
                           const BT::NodeConfiguration& conf);

    static BT::PortsList ProvidedBasicPorts(BT::PortsList addition) {
        BT::PortsList basic = {
            BT::InputPort<std::string>("server_name", "Action server name"),
            BT::InputPort<std::chrono::milliseconds>("server_timeout"),
        };
        basic.insert(addition.begin(), addition.end());
        return basic;
    }

    static BT::PortsList ProvidedPorts() { return ProvidedBasicPorts({}); }

    BT::NodeStatus tick() override;
    void halt() override;

protected:
    virtual void OnTick() {}
    virtual void OnWaitForResult(
        std::shared_ptr<const Feedback> /*feedback*/) {}
    virtual BT::NodeStatus OnSuccess();
    virtual BT::NodeStatus OnAborted();
    virtual BT::NodeStatus OnCancelled();
    virtual void OnTimeout() {}

    /** When true, completes on the first tick via bt_context (no autolink client). */
    virtual bool ExecuteInProcess(WrappedResult& result) {
        (void)result;
        return false;
    }

    Goal goal_;
    WrappedResult result_;
    std::shared_ptr<GoalHandle> goal_handle_;
    bool goal_updated_{false};

    mutable std::shared_ptr<autolink::Node> node_;
    mutable std::shared_ptr<ActionClient> action_client_;
    std::string action_name_;

    std::chrono::milliseconds server_timeout_{20000};
    std::chrono::milliseconds wait_for_server_timeout_{1000};

    bool should_send_goal_{true};
    bool goal_result_available_{false};

private:
    void SendGoal();
    bool ShouldCancelGoal() const;
    bool IsGoalAcceptancePending() const;
    bool IsGoalAcceptanceComplete(std::chrono::milliseconds elapsed);

    std::shared_ptr<autolink::Node> GetNode() const;
    std::shared_ptr<ActionClient> GetClient();

    std::shared_future<std::shared_ptr<GoalHandle>> goal_accept_future_;
    std::shared_future<WrappedResult> result_future_;
    std::chrono::steady_clock::time_point goal_sent_time_;
    std::shared_ptr<const Feedback> feedback_;
};

template <typename ActionT>
BtActionNode<ActionT>::BtActionNode(
    const std::string& xml_tag_name, const std::string& action_name,
    const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name) {
    getInput("server_timeout", server_timeout_);
    if (config().blackboard) {
        config().blackboard->get("wait_for_service_timeout",
                                wait_for_server_timeout_);
    }
    std::string remapped;
    if (getInput("server_name", remapped) && !remapped.empty()) {
        action_name_ = remapped;
    }
}

template <typename ActionT>
std::shared_ptr<autolink::Node> BtActionNode<ActionT>::GetNode()
    const {
    if (node_) {
        return node_;
    }
    if (!config().blackboard) {
        return nullptr;
    }
    config().blackboard->get(kBlackboardAutolinkNodeKey, node_);
    return node_;
}

template <typename ActionT>
std::shared_ptr<typename BtActionNode<ActionT>::ActionClient>
BtActionNode<ActionT>::GetClient() {
    if (action_client_) {
        return action_client_;
    }
    auto node = GetNode();
    if (!node) {
        return nullptr;
    }
    action_client_ = autolink::action::CreateClient<ActionT>(node, action_name_);
    const auto deadline = std::chrono::steady_clock::now() + wait_for_server_timeout_;
    while (autolink::OK() && !action_client_->ActionServerIsReady()) {
        if (std::chrono::steady_clock::now() > deadline) {
            AERROR << "Action server '" << action_name_
                   << "' not available after "
                   << wait_for_server_timeout_.count() << " ms";
            return nullptr;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return action_client_;
}

template <typename ActionT>
void BtActionNode<ActionT>::SendGoal() {
    goal_result_available_ = false;
    result_future_ = {};
    result_ = WrappedResult{};
    feedback_.reset();

    auto client = GetClient();
    if (!client) {
        throw std::runtime_error("action client unavailable");
    }

    typename ActionClient::SendGoalOptions options;
    options.feedback_callback =
        [this](std::shared_ptr<GoalHandle>,
               std::shared_ptr<const Feedback> feedback) {
            feedback_ = feedback;
        };

    goal_accept_future_ = client->AsyncSendGoal(goal_, options);
    goal_sent_time_ = std::chrono::steady_clock::now();
}

template <typename ActionT>
bool BtActionNode<ActionT>::IsGoalAcceptancePending() const {
    return goal_accept_future_.valid() && !goal_handle_;
}

template <typename ActionT>
bool BtActionNode<ActionT>::IsGoalAcceptanceComplete(
    std::chrono::milliseconds elapsed) {
    if (!goal_accept_future_.valid()) {
        return true;
    }
    if (goal_accept_future_.wait_for(std::chrono::milliseconds(0)) !=
        std::future_status::ready) {
        return elapsed >= server_timeout_;
    }
    goal_handle_ = goal_accept_future_.get();
    goal_accept_future_ = {};
    return true;
}

template <typename ActionT>
bool BtActionNode<ActionT>::ShouldCancelGoal() const {
    if (status() != BT::NodeStatus::RUNNING || !goal_handle_) {
        return false;
    }
    const auto status = goal_handle_->GetGoalStatus();
    return status == autolink::action::GoalStatus::ACCEPTED ||
           status == autolink::action::GoalStatus::EXECUTING;
}

template <typename ActionT>
BT::NodeStatus BtActionNode<ActionT>::tick() {
    if (!BT::isStatusActive(status())) {
        should_send_goal_ = true;
        goal_ = Goal{};
        result_ = WrappedResult{};
        goal_handle_.reset();
        goal_result_available_ = false;
        result_future_ = {};
        OnTick();
        setStatus(BT::NodeStatus::RUNNING);
        if (!should_send_goal_) {
            return BT::NodeStatus::FAILURE;
        }
        WrappedResult in_process_result;
        if (ExecuteInProcess(in_process_result)) {
            result_ = in_process_result;
            goal_result_available_ = true;
        } else {
            try {
                SendGoal();
            } catch (const std::exception& ex) {
                AERROR << "BtActionNode " << action_name_
                       << " send goal failed: " << ex.what();
                OnTimeout();
                return BT::NodeStatus::FAILURE;
            }
        }
    }

    if (WaitIfPaused(config())) {
        return BT::NodeStatus::RUNNING;
    }

    if (IsCancelRequested(config()) && ShouldCancelGoal()) {
        if (auto client = GetClient()) {
            client->AsyncCancelGoal(goal_handle_);
        }
    }

    try {
        if (IsGoalAcceptancePending()) {
            const auto elapsed =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - goal_sent_time_);
            if (!IsGoalAcceptanceComplete(elapsed)) {
                return BT::NodeStatus::RUNNING;
            }
            if (!goal_handle_) {
                OnTimeout();
                return BT::NodeStatus::FAILURE;
            }
            result_future_ = goal_handle_->AsyncGetResult();
        }

        if (!goal_result_available_ && result_future_.valid()) {
            if (result_future_.wait_for(std::chrono::milliseconds(0)) ==
                std::future_status::ready) {
                result_ = result_future_.get();
                goal_result_available_ = true;
            } else if (goal_handle_) {
                OnWaitForResult(feedback_);
                if (goal_updated_) {
                    goal_updated_ = false;
                    SendGoal();
                    return BT::NodeStatus::RUNNING;
                }
                return BT::NodeStatus::RUNNING;
            }
        }

        if (!goal_result_available_) {
            return BT::NodeStatus::RUNNING;
        }
    } catch (const std::exception& ex) {
        AERROR << "BtActionNode " << action_name_ << ": " << ex.what();
        return BT::NodeStatus::FAILURE;
    }

    goal_handle_.reset();
    switch (result_.code) {
        case autolink::action::ResultCode::SUCCEEDED:
            return OnSuccess();
        case autolink::action::ResultCode::ABORTED:
            return OnAborted();
        case autolink::action::ResultCode::CANCELED:
            return OnCancelled();
        default:
            return BT::NodeStatus::FAILURE;
    }
}

template <typename ActionT>
void BtActionNode<ActionT>::halt() {
    if (ShouldCancelGoal()) {
        if (auto client = GetClient()) {
            client->AsyncCancelGoal(goal_handle_);
        }
        OnCancelled();
    }
    goal_handle_.reset();
    goal_accept_future_ = {};
    result_future_ = {};
    goal_result_available_ = false;
    resetStatus();
}

template <typename ActionT>
BT::NodeStatus BtActionNode<ActionT>::OnSuccess() {
    return BT::NodeStatus::SUCCESS;
}

template <typename ActionT>
BT::NodeStatus BtActionNode<ActionT>::OnAborted() {
    return BT::NodeStatus::FAILURE;
}

template <typename ActionT>
BT::NodeStatus BtActionNode<ActionT>::OnCancelled() {
    return BT::NodeStatus::SUCCESS;
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

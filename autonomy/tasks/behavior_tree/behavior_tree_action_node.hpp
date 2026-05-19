/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "autonomy/common/logging.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "autonomy/tasks/behavior_tree/json_utils.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/json_export.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

enum class GoalStatus : uint8_t {
    UNKNOWN = 0,
    ACCEPTED,
    EXECUTING,
    CANCELING,
    SUCCEEDED,
    CANCELED,
    ABORTED,
};

enum class ResultCode : uint8_t {
    UNKNOWN = 0,
    SUCCEEDED,
    CANCELED,
    ABORTED,
};

template <typename ActionT>
struct WrappedResult {
    uint64_t goal_id = 0;
    ResultCode code = ResultCode::UNKNOWN;
    std::shared_ptr<typename ActionT::Result> result;
};

template <typename ActionT>
class ClientGoalHandle
{
public:
    explicit ClientGoalHandle(uint64_t goal_id)
        : goal_id_(goal_id), status_(GoalStatus::ACCEPTED) {}

    uint64_t GetGoalId() const {
        return goal_id_;
    }

    GoalStatus GetStatus() const {
        return status_;
    }

    void SetStatus(GoalStatus status) {
        status_ = status;
    }

    bool valid() const {
        return goal_id_ != 0;
    }

private:
    uint64_t goal_id_;
    GoalStatus status_;
};

template <typename ActionT>
struct SendGoalOptions {
    std::function<void(const WrappedResult<ActionT>&)> result_callback;
    std::function<void(std::shared_ptr<ClientGoalHandle<ActionT>>,
                       std::shared_ptr<const typename ActionT::Feedback>)>
        feedback_callback;
};

template <typename ActionT>
class ActionClient
{
public:
    explicit ActionClient(std::string action_name)
        : action_name_(std::move(action_name)) {}

    bool ActionServerIsReady() const {
        return true;
    }

    std::shared_future<std::shared_ptr<ClientGoalHandle<ActionT>>> AsyncSendGoal(
        const typename ActionT::Goal& /*goal*/,
        SendGoalOptions<ActionT> options) {
        auto promise = std::make_shared<
            std::promise<std::shared_ptr<ClientGoalHandle<ActionT>>>>();
        auto shared_future = promise->get_future().share();
        const uint64_t goal_id = ++goal_id_counter_;
        auto handle = std::make_shared<ClientGoalHandle<ActionT>>(goal_id);
        handle->SetStatus(GoalStatus::EXECUTING);
        promise->set_value(handle);
        if (options.result_callback) {
            std::thread([options = std::move(options), goal_id]() {
                WrappedResult<ActionT> wrapped;
                wrapped.goal_id = goal_id;
                wrapped.code = ResultCode::SUCCEEDED;
                wrapped.result = std::make_shared<typename ActionT::Result>();
                options.result_callback(wrapped);
            }).detach();
        }
        return shared_future;
    }

    std::shared_future<WrappedResult<ActionT>> AsyncGetResult(
        const std::shared_ptr<ClientGoalHandle<ActionT>>& /*handle*/) {
        WrappedResult<ActionT> wrapped;
        wrapped.code = ResultCode::SUCCEEDED;
        wrapped.result = std::make_shared<typename ActionT::Result>();
        std::promise<WrappedResult<ActionT>> promise;
        promise.set_value(wrapped);
        return promise.get_future().share();
    }

    std::shared_future<bool> AsyncCancelGoal(
        const std::shared_ptr<ClientGoalHandle<ActionT>>& handle) {
        if (handle) {
            handle->SetStatus(GoalStatus::CANCELED);
        }
        std::promise<bool> promise;
        promise.set_value(true);
        return promise.get_future().share();
    }

    static std::shared_ptr<ActionClient<ActionT>> Get(const std::string& name) {
        static std::mutex mutex;
        static std::unordered_map<std::string,
                                std::shared_ptr<ActionClient<ActionT>>>
            clients;
        std::lock_guard<std::mutex> lock(mutex);
        auto& client = clients[name];
        if (!client) {
            client = std::make_shared<ActionClient<ActionT>>(name);
        }
        return client;
    }

private:
    std::string action_name_;
    std::atomic<uint64_t> goal_id_counter_{0};
};

using namespace std::chrono_literals;  // NOLINT

template <class ActionT>
class BtActionNode : public BT::ActionNodeBase
{
public:
    BtActionNode(const std::string& xml_tag_name,
                 const std::string& action_name,
                 const BT::NodeConfiguration& conf)
        : BT::ActionNodeBase(xml_tag_name, conf),
          action_name_(action_name),
          should_send_goal_(true) {
        auto bt_loop_duration =
            config().blackboard->template get<std::chrono::milliseconds>(
                "bt_loop_duration");
        if (!GetInputPortOrBlackboard(*this, *config().blackboard,
                                      "server_timeout", server_timeout_)) {
            server_timeout_ = std::chrono::milliseconds(10);
        }
        wait_for_service_timeout_ =
            config().blackboard->template get<std::chrono::milliseconds>(
                "wait_for_service_timeout");

        max_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(
            bt_loop_duration * 0.5);

        goal_ = typename ActionT::Goal();
        result_ = WrappedResult<ActionT>();

        std::string remapped_action_name;
        if (getInput("server_name", remapped_action_name)) {
            action_name_ = remapped_action_name;
        }
        action_client_ = ActionClient<ActionT>::Get(action_name_);
        createActionClient(action_name_);

        ADEBUG << xml_tag_name.c_str() << " BtActionNode initialized";
    }

    BtActionNode() = delete;

    virtual ~BtActionNode() = default;

    void createActionClient(const std::string& action_name) {
        ADEBUG << "Waiting for \"" << action_name << "\" action";
        if (!action_client_->ActionServerIsReady()) {
            AERROR << "\"" << action_name << "\" action not available";
            throw std::runtime_error(std::string("Action ") + action_name +
                                     " not available");
        }
    }

    static BT::PortsList providedBasicPorts(BT::PortsList addition) {
        BT::PortsList basic = {
            BT::InputPort<std::string>("server_name", "Action name"),
            BT::InputPort<std::chrono::milliseconds>("server_timeout")};
        basic.insert(addition.begin(), addition.end());
        return basic;
    }

    static BT::PortsList providedPorts() {
        return providedBasicPorts({});
    }

    virtual void on_tick() {}

    virtual void on_wait_for_result(
        std::shared_ptr<const typename ActionT::Feedback> /*feedback*/) {}

    virtual BT::NodeStatus on_success() {
        return BT::NodeStatus::SUCCESS;
    }

    virtual BT::NodeStatus on_aborted() {
        return BT::NodeStatus::FAILURE;
    }

    virtual BT::NodeStatus on_cancelled() {
        return BT::NodeStatus::SUCCESS;
    }

    virtual void on_timeout() {}

    BT::NodeStatus tick() override {
        if (!BT::isStatusActive(status())) {
            should_send_goal_ = true;
            goal_ = typename ActionT::Goal();
            result_ = WrappedResult<ActionT>();
            on_tick();
            setStatus(BT::NodeStatus::RUNNING);

            if (!should_send_goal_) {
                return BT::NodeStatus::FAILURE;
            }
            send_new_goal();
        }

        try {
            if (future_goal_handle_) {
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<
                    std::chrono::milliseconds>(now - time_goal_sent_);
                if (!is_future_goal_handle_complete(elapsed)) {
                    if (elapsed < server_timeout_) {
                        return BT::NodeStatus::RUNNING;
                    }
                    AWARN << "Timed out waiting for action to acknowledge goal: "
                          << action_name_;
                    future_goal_handle_.reset();
                    on_timeout();
                    return BT::NodeStatus::FAILURE;
                }
            }

            if (!goal_result_available_) {
                on_wait_for_result(feedback_);
                feedback_.reset();

                if (goal_handle_) {
                    auto goal_status = goal_handle_->GetStatus();
                    if (goal_updated_ &&
                        (goal_status == GoalStatus::EXECUTING ||
                         goal_status == GoalStatus::ACCEPTED)) {
                        goal_updated_ = false;
                        send_new_goal();
                        auto now = std::chrono::steady_clock::now();
                        auto elapsed = std::chrono::duration_cast<
                            std::chrono::milliseconds>(now - time_goal_sent_);
                        if (!is_future_goal_handle_complete(elapsed)) {
                            if (elapsed < server_timeout_) {
                                return BT::NodeStatus::RUNNING;
                            }
                            AWARN << "Timed out waiting for action to "
                                     "acknowledge goal: "
                                  << action_name_;
                            future_goal_handle_.reset();
                            on_timeout();
                            return BT::NodeStatus::FAILURE;
                        }
                    }
                }

                if (!goal_result_available_) {
                    return BT::NodeStatus::RUNNING;
                }
            }
        } catch (const std::runtime_error& e) {
            if (e.what() == std::string("send_goal failed") ||
                e.what() ==
                    std::string("Goal was rejected by the action server")) {
                return BT::NodeStatus::FAILURE;
            }
            throw;
        }

        BT::NodeStatus bt_status;
        switch (result_.code) {
            case ResultCode::SUCCEEDED:
                bt_status = on_success();
                break;
            case ResultCode::ABORTED:
                bt_status = on_aborted();
                break;
            case ResultCode::CANCELED:
                bt_status = on_cancelled();
                break;
            default:
                throw std::logic_error(
                    "BtActionNode::tick: invalid status value");
        }

        goal_handle_.reset();
        return bt_status;
    }

    void halt() override {
        if (should_cancel_goal()) {
            auto future_cancel = action_client_->AsyncCancelGoal(goal_handle_);
            if (future_cancel.wait_for(server_timeout_) !=
                std::future_status::ready) {
                AERROR << "Failed to cancel action for " << action_name_;
            }
            on_cancelled();
        }
        resetStatus();
    }

protected:
    bool should_cancel_goal() {
        if (status() != BT::NodeStatus::RUNNING) {
            return false;
        }
        if (!goal_handle_) {
            return false;
        }
        const auto goal_status = goal_handle_->GetStatus();
        return goal_status == GoalStatus::ACCEPTED ||
               goal_status == GoalStatus::EXECUTING;
    }

    void send_new_goal() {
        goal_result_available_ = false;
        SendGoalOptions<ActionT> send_goal_options;
        send_goal_options.result_callback =
            [this](const WrappedResult<ActionT>& result) {
                if (future_goal_handle_ && future_goal_handle_->valid()) {
                    return;
                }
                if (goal_handle_ &&
                    goal_handle_->GetGoalId() == result.goal_id) {
                    goal_result_available_ = true;
                    result_ = result;
                    emitWakeUpSignal();
                }
            };
        send_goal_options.feedback_callback =
            [this](std::shared_ptr<ClientGoalHandle<ActionT>> /*handle*/,
                   const std::shared_ptr<const typename ActionT::Feedback>
                       feedback) {
                feedback_ = feedback;
                emitWakeUpSignal();
            };

        future_goal_handle_ = std::make_shared<std::shared_future<
            std::shared_ptr<ClientGoalHandle<ActionT>>>>(
            action_client_->AsyncSendGoal(goal_, send_goal_options));
        time_goal_sent_ = std::chrono::steady_clock::now();
    }

    bool is_future_goal_handle_complete(std::chrono::milliseconds& elapsed) {
        if (!future_goal_handle_ || !future_goal_handle_->valid()) {
            return false;
        }

        auto remaining = server_timeout_ - elapsed;
        if (remaining <= std::chrono::milliseconds(0)) {
            future_goal_handle_.reset();
            return false;
        }

        auto timeout = remaining > max_timeout_ ? max_timeout_ : remaining;
        auto wait_result = future_goal_handle_->wait_for(timeout);
        elapsed += timeout;

        if (wait_result == std::future_status::deferred) {
            future_goal_handle_.reset();
            throw std::runtime_error("send_goal failed");
        }

        if (wait_result == std::future_status::ready) {
            goal_handle_ = future_goal_handle_->get();
            future_goal_handle_.reset();
            if (!goal_handle_) {
                throw std::runtime_error(
                    "Goal was rejected by the action server");
            }
            return true;
        }

        return false;
    }

    void increment_recovery_count() {
        int recovery_count = 0;
        [[maybe_unused]] auto res = config().blackboard->get(
            "number_recoveries", recovery_count);  // NOLINT
        recovery_count += 1;
        config().blackboard->set("number_recoveries",
                                 recovery_count);  // NOLINT
    }

    std::string action_name_;
    std::shared_ptr<ActionClient<ActionT>> action_client_;
    typename ActionT::Goal goal_;
    bool goal_updated_{false};
    bool goal_result_available_{false};
    std::shared_ptr<ClientGoalHandle<ActionT>> goal_handle_;
    WrappedResult<ActionT> result_;
    std::shared_ptr<const typename ActionT::Feedback> feedback_;
    std::chrono::milliseconds server_timeout_;
    std::chrono::milliseconds max_timeout_;
    std::chrono::milliseconds wait_for_service_timeout_;
    std::shared_ptr<std::shared_future<std::shared_ptr<ClientGoalHandle<ActionT>>>>
        future_goal_handle_;
    std::chrono::steady_clock::time_point time_goal_sent_;
    bool should_send_goal_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

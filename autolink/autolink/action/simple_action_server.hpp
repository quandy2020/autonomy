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

#include <algorithm>
#include <cctype>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>

#include "autolink/action/create_server.hpp"
#include "autolink/action/server.hpp"
#include "autolink/autolink.hpp"
#include "autolink/common/macros.hpp"

namespace autolink {
namespace action {

/**
 * @class autolink::action::SimpleActionServer
 * @brief An action server wrapper to make applications simpler using Actions
 */
template <typename ActionT>
class SimpleActionServer
{
public:
    /**
     * Define SimpleActionServer::SharedPtr type
     */
    AUTOLINK_SHARED_PTR_DEFINITIONS(SimpleActionServer)

    /**
     * @brief Callback function to complete main work. This should itself deal
     * with its own exceptions, but if for some reason one is thrown, it will be
     * caught in SimpleActionServer and terminate the action itself.
     */
    using ExecuteCallback = std::function<void()>;

    /**
     * @brief Callback function to notify the user that an exception was thrown
     * that the simple action server caught (or another failure) and the action
     * was terminated. To avoid using, catch exceptions in your application such
     * that the SimpleActionServer will never need to terminate based on failed
     * action ExecuteCallback.
     */
    using CompletionCallback = std::function<void()>;

    /**
     * @brief Optional callback to accept or reject a goal. If set, HandleGoal
     * calls it; if not set, goals are accepted when server is active.
     */
    using GoalCallback = std::function<autolink::action::GoalResponse(
        const autolink::action::GoalUUID&,
        std::shared_ptr<const typename ActionT::Goal>)>;

    /**
     * @brief A constructor for SimpleActionServer
     * @param node Shared pointer to node to make actions
     * @param action_name Name of the action to call
     * @param execute_callback Execution callback function of Action
     * @param completion_callback Completion callback function
     * @param server_timeout Timeout to react to stop or preemption requests
     * @param realtime Whether the action server's worker thread should have
     * elevated prioritization (soft realtime)
     *
     * The server is activated in this constructor and deactivated in the
     * destructor. @ref Activate and @ref Deactivate remain available for manual
     * lifecycle control if needed.
     */
    explicit SimpleActionServer(
        std::shared_ptr<autolink::Node> node, const std::string& action_name,
        ExecuteCallback execute_callback,
        CompletionCallback completion_callback = nullptr,
        std::chrono::milliseconds server_timeout =
            std::chrono::milliseconds(500),
        const bool realtime = false)
        : node_(node),
          action_name_(action_name),
          execute_callback_(execute_callback),
          completion_callback_(completion_callback),
          server_timeout_(server_timeout),
          use_realtime_prioritization_(realtime) {
        action_server_ = autolink::action::CreateServer<ActionT>(
            node_, action_name_,
            std::bind(&SimpleActionServer::HandleGoal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&SimpleActionServer::HandleCancel, this,
                      std::placeholders::_1),
            std::bind(&SimpleActionServer::HandleAccepted, this,
                      std::placeholders::_1));
        Activate();
    }

    ~SimpleActionServer() {
        Deactivate();
    }

    /**
     * @brief handle the goal requested: accept or reject.
     * This implementation always accepts when the server is active.
     * @param uuid Goal ID
     * @param Goal A shared pointer to the specific goal
     * @return GoalResponse response of the goal processed
     */
    autolink::action::GoalResponse HandleGoal(
        const autolink::action::GoalUUID& uuid,
        std::shared_ptr<const typename ActionT::Goal> goal) {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!server_active_) {
            AINFO << "Action server is inactive. Rejecting the goal.";
            return autolink::action::GoalResponse::REJECT;
        }

        if (goal_callback_) {
            return goal_callback_(uuid, goal);
        }

        DebugMsg("Received request for goal acceptance");
        return autolink::action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /**
     * @brief Set optional callback to accept or reject goals by content.
     * If not set, all goals are accepted when server is active.
     */
    void SetGoalCallback(GoalCallback cb) {
        goal_callback_ = std::move(cb);
    }

    /**
     * @brief Accepts cancellation requests of action server.
     * @param handle A server goal handle to cancel
     * @return CancelResponse response of the goal cancelled
     */
    autolink::action::CancelResponse HandleCancel(
        const std::shared_ptr<autolink::action::ServerGoalHandle<ActionT>>
            handle) {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!handle || !handle->IsActive()) {
            WarnMsg(
                "Received request for goal cancellation,"
                "but the handle is inactive, so reject the request");
            return autolink::action::CancelResponse::REJECT;
        }

        DebugMsg("Received request for goal cancellation");
        return autolink::action::CancelResponse::ACCEPT;
    }

    /**
     * @brief Sets thread priority level
     */
    void SetSoftRealTimePriority() {
        if (use_realtime_prioritization_) {
            sched_param sch;
            sch.sched_priority = 49;
            if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
                std::string errmsg(
                    "Cannot set as real-time thread. Users must set: "
                    "<username> hard rtprio 99 and "
                    "<username> soft rtprio 99 in /etc/security/limits.conf to "
                    "enable "
                    "realtime prioritization! Error: ");
                throw std::runtime_error(errmsg + std::strerror(errno));
            }
            DebugMsg("Soft realtime prioritization successfully set!");
        }
    }

    /**
     * @brief Handles accepted goals and adds to preempted queue to switch to
     * @param handle A server goal handle
     */
    void HandleAccepted(
        const std::shared_ptr<autolink::action::ServerGoalHandle<ActionT>>
            handle) {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        DebugMsg("Receiving a new goal");

        if (IsActive(current_handle_) || IsRunning()) {
            DebugMsg(
                "An older goal is active, moving the new goal to a pending "
                "slot.");

            if (IsActive(pending_handle_)) {
                DebugMsg(
                    "The pending slot is occupied. The previous pending goal "
                    "will be terminated and replaced.");
                Terminate(pending_handle_);
            }
            pending_handle_ = handle;
        } else {
            if (IsActive(pending_handle_)) {
                // Shouldn't reach a state with a pending goal but no current
                // one.
                ErrorMsg(
                    "Forgot to handle a preemption. Terminating the pending "
                    "goal.");
                Terminate(pending_handle_);
            }

            current_handle_ = handle;

            // Return quickly to avoid blocking the executor, so spin up a new
            // thread
            DebugMsg("Executing goal asynchronously.");
            if (current_handle_) {
                current_handle_->Execute();
            }
            execution_future_ = std::async(std::launch::async, [this]() {
                SetSoftRealTimePriority();
                Work();
            });
        }
    }

    /**
     * @brief Computed background work and processes stop requests
     */
    void Work() {
        while (autolink::OK() && !stop_execution_ &&
               (IsActive(current_handle_) || IsActive(pending_handle_))) {
            {
                std::lock_guard<std::recursive_mutex> lock(update_mutex_);
                if (IsActive(pending_handle_) &&
                    (!IsActive(current_handle_) ||
                     current_handle_ != pending_handle_)) {
                    DebugMsg(
                        "Preemption: switching to pending goal before execute");
                    AcceptPendingGoal();
                }
            }

            if (!IsActive(current_handle_)) {
                DebugMsg("Done processing available goals.");
                break;
            }

            DebugMsg("Executing the goal...");
            try {
                execute_callback_();
            } catch (std::exception& ex) {
                AERROR << "Action server failed while executing action "
                          "callback: "
                       << ex.what();
                TerminateAll();
                if (completion_callback_) {
                    completion_callback_();
                }
                return;
            }

            DebugMsg("Blocking processing of new goal handles.");
            std::lock_guard<std::recursive_mutex> lock(update_mutex_);

            if (stop_execution_) {
                WarnMsg("Stopping the thread per request.");
                TerminateAll();
                if (completion_callback_) {
                    completion_callback_();
                }
                break;
            }

            if (IsActive(pending_handle_)) {
                // Preempt in flight: old goal is finished for this round;
                // switch at loop head so execute_callback_ is never run again
                // for it.
                DebugMsg("Preemption pending; will switch before next execute");
            } else if (IsActive(current_handle_)) {
                WarnMsg("Current goal was not completed successfully.");
                Terminate(current_handle_);
                if (completion_callback_) {
                    completion_callback_();
                }
            } else {
                DebugMsg("Done processing available goals.");
                break;
            }
        }
        DebugMsg("Worker thread done.");
    }

    /**
     * @brief Active action server
     */
    void Activate() {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        server_active_ = true;
        stop_execution_ = false;
    }

    /**
     * @brief Deactivate action server
     */
    void Deactivate() {
        DebugMsg("Deactivating...");
        {
            std::lock_guard<std::recursive_mutex> lock(update_mutex_);
            server_active_ = false;
            stop_execution_ = true;
        }

        if (!execution_future_.valid()) {
            return;
        }

        if (IsRunning()) {
            WarnMsg(
                "Requested to deactivate server but goal is still "
                "executing."
                " Should check if action server is running before "
                "deactivating.");
        }

        using namespace std::chrono;  // NOLINT
        auto start_time = steady_clock::now();
        while (execution_future_.wait_for(milliseconds(100)) !=
               std::future_status::ready) {
            InfoMsg("Waiting for async process to finish.");
            if (steady_clock::now() - start_time >= server_timeout_) {
                TerminateAll();
                if (completion_callback_) {
                    completion_callback_();
                }
                ErrorMsg(
                    "Action callback is still running and missed deadline "
                    "to stop");
            }
        }

        DebugMsg("Deactivation completed.");
    }

    /**
     * @brief Whether the action server is munching on a goal
     * @return bool If its running or not
     */
    bool IsRunning() {
        return execution_future_.valid() &&
               (execution_future_.wait_for(std::chrono::milliseconds(0)) ==
                std::future_status::timeout);
    }

    /**
     * @brief Whether the action server is active or not
     * @return bool If its active or not
     */
    bool IsServerActive() {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        return server_active_;
    }

    /**
     * @brief Whether a newer accepted goal is waiting to replace the current
     * one (same condition @ref Work uses before each execute).
     * @return bool If preemption is pending or not
     */
    bool IsPreemptRequested() const {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        return IsActive(pending_handle_) &&
               (!IsActive(current_handle_) ||
                current_handle_ != pending_handle_);
    }

    /**
     * @brief Accept pending goals
     * @return Goal Ptr to the  goal that's going to be accepted
     */
    const std::shared_ptr<const typename ActionT::Goal> AcceptPendingGoal() {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!pending_handle_ || !pending_handle_->IsActive()) {
            ErrorMsg("Attempting to get pending goal when not available");
            return std::shared_ptr<const typename ActionT::Goal>();
        }

        if (IsActive(current_handle_) && current_handle_ != pending_handle_) {
            DebugMsg("Cancelling the previous goal");
            current_handle_->Abort(EmptyResult());
        }

        current_handle_ = pending_handle_;
        pending_handle_.reset();

        DebugMsg("Preempted goal");
        if (!current_handle_) {
            return std::shared_ptr<const typename ActionT::Goal>();
        }
        if (current_handle_->IsCanceling()) {
            DebugMsg(
                "Pending goal was canceled before run; completing as CANCELED");
            Terminate(current_handle_, EmptyResult());
            return std::shared_ptr<const typename ActionT::Goal>();
        }
        current_handle_->Execute();
        return current_handle_->GetGoal();
    }

    /**
     * @brief Terminate pending goals
     */
    void TerminatePendingGoal() {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (!pending_handle_ || !pending_handle_->IsActive()) {
            ErrorMsg("Attempting to terminate pending goal when not available");
            return;
        }
        Terminate(pending_handle_);
        DebugMsg("Pending goal terminated");
    }

    /**
     * @brief Get the current goal object
     * @return Goal Ptr to the  goal that's being processed currently
     */
    const std::shared_ptr<const typename ActionT::Goal> GetCurrentGoal() const {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (!IsActive(current_handle_)) {
            ErrorMsg("A goal is not available or has reached a final state");
            return std::shared_ptr<const typename ActionT::Goal>();
        }
        return current_handle_->GetGoal();
    }

    const autolink::action::GoalUUID GetCurrentGoalID() const {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (!IsActive(current_handle_)) {
            ErrorMsg("A goal is not available or has reached a final state");
            autolink::action::GoalUUID empty_uuid{};
            return empty_uuid;
        }
        return current_handle_->GetGoalId();
    }

    /**
     * @brief Get the pending goal object
     * @return Goal Ptr to the goal that's pending
     */
    const std::shared_ptr<const typename ActionT::Goal> GetPendingGoal() const {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!pending_handle_ || !pending_handle_->IsActive()) {
            ErrorMsg("Attempting to get pending goal when not available");
            return std::shared_ptr<const typename ActionT::Goal>();
        }

        return pending_handle_->GetGoal();
    }

    /**
     * @brief Whether or not a cancel command has come in
     * @return bool Whether a cancel command has been requested or not
     */
    bool IsCancelRequested() const {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (current_handle_ == nullptr) {
            ErrorMsg("Checking for cancel but current goal is not available");
            return false;
        }
        // Always reflect the executing goal; pending may belong to another
        // client.
        return current_handle_->IsCanceling();
    }

    /**
     * @brief Terminate all pending and active actions
     * @param result A result object to send to the terminated actions
     */
    void TerminateAll(std::shared_ptr<typename ActionT::Result> result =
                          std::make_shared<typename ActionT::Result>()) {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        Terminate(current_handle_, result);
        Terminate(pending_handle_, result);
    }

    /**
     * @brief Terminate the active action
     * @param result A result object to send to the terminated action
     */
    void TerminateCurrent(std::shared_ptr<typename ActionT::Result> result =
                              std::make_shared<typename ActionT::Result>()) {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        Terminate(current_handle_, result);
    }

    /**
     * @brief Return success of the active action
     * @param result A result object to send to the terminated actions
     */
    void SucceededCurrent(std::shared_ptr<typename ActionT::Result> result =
                              std::make_shared<typename ActionT::Result>()) {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (IsActive(current_handle_)) {
            DebugMsg("Setting succeed on current goal.");
            current_handle_->Succeed(result);
            current_handle_.reset();
        }
    }

    /**
     * @brief Publish feedback to the action server clients
     * @param feedback A feedback object to send to the clients
     */
    void PublishFeedback(std::shared_ptr<typename ActionT::Feedback> feedback) {
        if (!IsActive(current_handle_)) {
            ErrorMsg(
                "Trying to publish feedback when the current goal handle "
                "is not active");
            return;
        }
        current_handle_->PublishFeedback(feedback);
    }

protected:
    std::shared_ptr<autolink::Node> node_;
    std::string action_name_;

    ExecuteCallback execute_callback_;
    CompletionCallback completion_callback_;
    GoalCallback goal_callback_;
    std::future<void> execution_future_;
    bool stop_execution_{false};
    bool use_realtime_prioritization_{false};

    mutable std::recursive_mutex update_mutex_;
    bool server_active_{false};
    std::chrono::milliseconds server_timeout_;

    std::shared_ptr<autolink::action::ServerGoalHandle<ActionT>>
        current_handle_;
    std::shared_ptr<autolink::action::ServerGoalHandle<ActionT>>
        pending_handle_;

    std::shared_ptr<autolink::action::Server<ActionT>> action_server_;

    /**
     * @brief Generate an empty result object for an action type
     */
    constexpr auto EmptyResult() const {
        return std::make_shared<typename ActionT::Result>();
    }

    /**
     * @brief Whether a given goal handle is currently active
     * @param handle Goal handle to check
     * @return Whether this goal handle is active
     */
    bool IsActive(
        const std::shared_ptr<autolink::action::ServerGoalHandle<ActionT>>
            handle) const {
        return handle != nullptr && handle->IsActive();
    }

    /**
     * @brief SFINAE (Substitution Failure Is Not An Error) to check
     *        for existence of error_code and error_msg in ActionT::Result
     *        This allows for ActionT::Result messages that do not contain
     *        an error_code or error_msg such as
     * test_msgs::action::Fibonacci
     */
    template <typename T, typename = void>
    struct has_error_msg : std::false_type {};

    template <typename T>
    struct has_error_msg<T, std::void_t<decltype(T::error_msg)>>
        : std::true_type {};

    template <typename T, typename = void>
    struct has_error_code : std::false_type {};

    template <typename T>
    struct has_error_code<T, std::void_t<decltype(T::error_code)>>
        : std::true_type {};

    /**
     * @brief Log error details if available
     * @param result the Results object to log the error details
     */
    template <typename T>
    void LogErrorDetailsIfAvailable(const T& result) {
        if constexpr (has_error_code<typename ActionT::Result>::value &&
                      has_error_msg<typename ActionT::Result>::value) {
            WarnMsg("Aborting handle. error_code:" +
                    std::to_string(result->error_code) + ", error_msg:'" +
                    result->error_msg + "'.");
        } else if constexpr (has_error_code<typename ActionT::Result>::value) {
            WarnMsg("Aborting handle. error_code:" +
                    std::to_string(result->error_code) + ".");
        } else {
            WarnMsg("Aborting handle.");
        }
    }

    /**
     * @brief Terminate a particular action with a result
     * @param handle goal handle to terminate
     * @param result the Results object to terminate the action with
     */
    void Terminate(
        std::shared_ptr<autolink::action::ServerGoalHandle<ActionT>>& handle,
        std::shared_ptr<typename ActionT::Result> result =
            std::make_shared<typename ActionT::Result>()) {
        if (!handle) {
            return;
        }

        if (IsActive(handle)) {
            if (handle->IsCanceling()) {
                InfoMsg("Client requested to cancel the goal. Cancelling.");
                handle->Canceled(result);
            } else {
                LogErrorDetailsIfAvailable(result);
                handle->Abort(result);
            }
            handle.reset();
        }
    }

    /**
     * @brief Info logging
     * @param msg The message to log
     */
    void InfoMsg(const std::string& msg) const {
        AINFO << action_name_ << "[ActionServer] " << msg;
    }

    /**
     * @brief Debug logging
     * @param msg The message to log
     */
    void DebugMsg(const std::string& msg) const {
        ADEBUG << action_name_ << "[ActionServer] " << msg;
    }

    /**
     * @brief Error logging
     * @param msg The message to log
     */
    void ErrorMsg(const std::string& msg) const {
        AERROR << action_name_ << "[ActionServer] " << msg;
    }

    /**
     * @brief Warn logging
     * @param msg The message to log
     */
    void WarnMsg(const std::string& msg) const {
        AWARN << action_name_ << "[ActionServer] " << msg;
    }
};

}  // namespace action
}  // namespace autolink

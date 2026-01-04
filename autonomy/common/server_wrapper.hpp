/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "autolink/service/service.hpp"
#include "autolink/common/log.hpp"

namespace autonomy {
namespace common {

/**
 * @brief Wrapper of cyber::Client which sends a topic with service name when
 * SendRequst is invoked.
 */
template <class ServiceT>
class ServiceWrapper
{
public:
    using ExecuteCallback = std::function<void()>;

    using CompletionCallback = std::function<void()>;

    ServiceWrapper(const std::shared_ptr<::autolink::Node>& node,
                   const std::string& service_name,
                   ExecuteCallback execute_callback,
                   CompletionCallback completion_callback = nullptr,
                   std::chrono::milliseconds server_timeout =
                       std::chrono::milliseconds(500))
        : service_name_(service_name),
          execute_callback_(execute_callback),
          completion_callback_(completion_callback),
          server_timeout_(server_timeout) {
        server_ = node->CreateService<typename ServiceT::Request,
                                      typename ServiceT::Response>(
            service_name,
            [this](const std::shared_ptr<typename ServiceT::Request>& request,
                   std::shared_ptr<typename ServiceT::Response>& response) {
                server_active_ = true;
                HandleAccepted(request, response);
            });
    }

    /**
     * @brief
     *
     * @param request
     * @return true
     * @return false
     */
    bool HandleAccepted(
        const std::shared_ptr<typename ServiceT::Request>& request,
        std::shared_ptr<typename ServiceT::Response>& response) {
        InfoMsg("Executing goal asynchronously.");
        execution_future_ =
            std::async(std::launch::async, [this]() { Work(); });
        return true;
    }

    /**
     * @brief Computed background work and processes stop requests
     */
    void Work() {
        while (autolink::OK() && !stop_execution_) {
            InfoMsg("Executing the goal...");
            try {
                InfoMsg("Executing the goal...1");
                execute_callback_();

                InfoMsg("Executing the goal 2");
            } catch (std::exception& ex) {
                LOG(ERROR)
                    << "Action server failed while executing action callback: "
                    << ex.what();
                completion_callback_();
                return;
            }

            if (stop_execution_) {
                WarnMsg("Stopping the thread per request.");
                completion_callback_();
                break;
            }
        }
        InfoMsg("Accept worker thread done.");
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
     * @brief Whether or not a cancel command has come in
     * @return bool Whether a cancel command has been requested or not
     */
    bool IsCancelRequested() const {
        return true;
    }

    /**
     * @brief Whether the action server has been asked to be preempted with a
     * new goal
     * @return bool If there's a preemption request or not
     */
    bool IsPreemptRequested() const {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        return preempt_requested_;
    }

    /**
     * @brief Terminate pending request
     */
    void TerminatePendingRequest() {}

    void TerminateCurrent() {}

    /**
     * @brief Get the current request object
     * @return request Ptr to the  goal that's being processed currently
     */
    const std::shared_ptr<const typename ServiceT::Request> GetCurrentRequest()
        const {
        return nullptr;
    }

    /**
     * @brief Get the pending request object
     * @return Request Ptr to the request that's pending
     */
    const std::shared_ptr<const typename ServiceT::Request> GetPendingRequest()
        const {
        return nullptr;
    }

protected:
    /**
     * @brief Info logging
     */
    void InfoMsg(const std::string& msg) const {
        LOG(INFO) << service_name_.c_str() << "[ActionServer] " << msg.c_str();
    }

    /**
     * @brief Error logging
     */
    void ErrorMsg(const std::string& msg) const {
        LOG(ERROR) << service_name_.c_str() << "[ActionServer] " << msg.c_str();
    }

    /**
     * @brief Warn logging
     */
    void WarnMsg(const std::string& msg) const {
        LOG(WARNING) << service_name_.c_str() << "[ActionServer] "
                     << msg.c_str();
    }

    /**
     * @brief Generate an empty result object for an action type
     */
    constexpr auto EmptyResult() const {
        return std::make_shared<typename ServiceT::Response>();
    }

    std::string service_name_;

    ExecuteCallback execute_callback_;
    CompletionCallback completion_callback_;
    std::future<void> execution_future_;
    bool stop_execution_{false};

    mutable std::recursive_mutex update_mutex_;
    bool server_active_{false};
    bool preempt_requested_{false};
    std::chrono::milliseconds server_timeout_;

    std::shared_ptr<::autolink::Service<typename ServiceT::Request,
                                        typename ServiceT::Response>>
        server_;
};

}  // namespace common
}  // namespace autonomy
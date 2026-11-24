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
 * @file action_endpoint.hpp
 * @brief Own one native coverage action worker and its lifecycle.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_ACTION_ENDPOINT_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_ACTION_ENDPOINT_HPP_

#include <cstdio>
#include <functional>
#include <thread>
#include "autolink/action/create_server.hpp"
#include "autonomy/task/navigation/coverage/action_endpoint_state.hpp"
#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/**
 * @brief Execute each accepted handle and drain its terminal result on
          * shutdown.
 */
template <class Action>
class ActionEndpoint {
 public:
  using Handle = autolink::action::ServerGoalHandle<Action>;
  using Execute = std::function<void(const std::shared_ptr<Handle>&,
                                    const std::function<bool()>&)>;
  /**
   * @brief Bind a native action with one in-flight goal and one worker.
   */
  ActionEndpoint(std::shared_ptr<autolink::Node> node, std::string name,
                 Execute execute) : execute_(std::move(execute)) {
    const auto state = state_;
    server_ = autolink::action::CreateServer<Action>(node, name,
      [state](const auto&, auto) {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (state->stopping || state->reserved) {
          // One goal runs at a time. Report the rejection on the node log so a
          // dropped click in the panel is not mistaken for a planner failure.
          std::fprintf(stderr,
                       "coverage: goal rejected, %s\n",
                       state->stopping ? "endpoint is stopping"
                                       : "another goal is still running");
          return autolink::action::GoalResponse::REJECT;
        }
        state->reserved = true;
        return autolink::action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](auto handle) {
        return handle && handle->IsActive() ? autolink::action::CancelResponse::ACCEPT
                                            : autolink::action::CancelResponse::REJECT;
      },
      [state](auto handle) {
        { std::lock_guard<std::mutex> lock(state->mutex); state->pending = std::move(handle); }
        state->wake.notify_all();
      });
    worker_ = std::thread([this] { Run(); });
  }
  ~ActionEndpoint() { Stop(); }
  /**
   * @brief Close admission and retain the server until accepted work
            * terminates.
   */
  void Stop() {
    state_->stopping = true;
    state_->wake.notify_all();
    if (worker_.joinable()) worker_.join();
    server_.reset();
  }
 private:
  /**
   * @brief Enter EXECUTING before invoking callbacks that publish terminal
            * results.
   */
  void Run() {
    const auto state = state_;
    for (;;) {
      std::shared_ptr<Handle> handle;
      {
        std::unique_lock<std::mutex> lock(state->mutex);
        state->wake.wait(lock, [&] { return state->pending || (state->stopping && !state->reserved); });
        if (!state->pending) return;
        handle = std::move(state->pending);
      }
      handle->Execute();
      try {
        execute_(handle, [state, handle] { return state->stopping || handle->IsCanceling(); });
      } catch (const std::exception& error) {
        auto result = std::make_shared<typename Action::Result>();
        result->set_error_code(proto::UNKNOWN);
        result->set_error_msg(error.what());
        if (handle->IsCanceling()) handle->Canceled(result);
        else if (handle->IsActive()) handle->Abort(result);
      }
      { std::lock_guard<std::mutex> lock(state->mutex); state->reserved = false; }
      state->wake.notify_all();
      if (state->stopping) return;
    }
  }
  Execute execute_;
  std::shared_ptr<ActionEndpointState<Action>> state_{std::make_shared<ActionEndpointState<Action>>()};
  std::shared_ptr<autolink::action::Server<Action>> server_;
  std::thread worker_;
};
}

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_ACTION_ENDPOINT_HPP_

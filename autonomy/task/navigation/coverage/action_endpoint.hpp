// SPDX-License-Identifier: Apache-2.0
/** @file action_endpoint.hpp
 * @brief Own one native coverage action worker and its lifecycle. */
#pragma once
#include <functional>
#include <thread>
#include "autolink/action/create_server.hpp"
#include "autonomy/task/navigation/coverage/action_endpoint_state.hpp"
#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/** @brief Execute each accepted handle and drain its terminal result on shutdown. */
template <class Action>
class ActionEndpoint {
 public:
  using Handle = autolink::action::ServerGoalHandle<Action>;
  using Execute = std::function<void(const std::shared_ptr<Handle>&,
                                    const std::function<bool()>&)>;
  /** @brief Bind a native action with one in-flight goal and one worker. */
  ActionEndpoint(std::shared_ptr<autolink::Node> node, std::string name,
                 Execute execute) : execute_(std::move(execute)) {
    const auto state = state_;
    server_ = autolink::action::CreateServer<Action>(node, name,
      [state](const auto&, auto) {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (state->stopping || state->reserved) return autolink::action::GoalResponse::REJECT;
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
  /** @brief Close admission and retain the server until accepted work terminates. */
  void Stop() {
    state_->stopping = true;
    state_->wake.notify_all();
    if (worker_.joinable()) worker_.join();
    server_.reset();
  }
 private:
  /** @brief Enter EXECUTING before invoking callbacks that publish terminal results. */
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

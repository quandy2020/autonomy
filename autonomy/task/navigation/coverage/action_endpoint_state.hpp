// SPDX-License-Identifier: Apache-2.0
/** @file action_endpoint_state.hpp
 * @brief Retain action handoff state until admitted callbacks have drained. */
#pragma once
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include "autolink/action/server_goal_handle.hpp"
namespace autonomy::task::coverage {
/** @brief Shared handoff ownership independent of the endpoint facade lifetime. */
template <class Action>
struct ActionEndpointState {
  std::mutex mutex;  ///< Serializes acceptance and worker handoff.
  std::condition_variable wake;  ///< Wakes the worker on acceptance or shutdown.
  std::shared_ptr<autolink::action::ServerGoalHandle<Action>> pending;
  bool reserved{false};  ///< An accepted goal awaits handoff or execution.
  std::atomic<bool> stopping{false};  ///< Refuse new goals and cancel owned work.
};
}

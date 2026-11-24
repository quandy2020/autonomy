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
 * @file action_endpoint_state.hpp
 * @brief Retain action handoff state until admitted callbacks have drained.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_ACTION_ENDPOINT_STATE_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_ACTION_ENDPOINT_STATE_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include "autolink/action/server_goal_handle.hpp"
namespace autonomy::task::coverage {
/**
 * @brief Shared handoff ownership independent of the endpoint facade
          * lifetime.
 */
template <class Action>
struct ActionEndpointState {
  std::mutex mutex;  ///< Serializes acceptance and worker handoff.
  std::condition_variable wake;  ///< Wakes the worker on acceptance or shutdown.
  std::shared_ptr<autolink::action::ServerGoalHandle<Action>> pending;
  bool reserved{false};  ///< An accepted goal awaits handoff or execution.
  std::atomic<bool> stopping{false};  ///< Refuse new goals and cancel owned work.
};
}

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_ACTION_ENDPOINT_STATE_HPP_

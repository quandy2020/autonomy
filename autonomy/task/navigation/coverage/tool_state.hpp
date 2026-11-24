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
 * @file tool_state.hpp
 * @brief Correlate tool command acknowledgments.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_TOOL_STATE_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_TOOL_STATE_HPP_

#include <condition_variable>
#include <mutex>

#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/**
 * @brief Shared acknowledgment state retained by native reader callbacks.
 *
 * The reader thread stores the latest acknowledgment and bumps the sequence;
 * the waiting worker compares the sequence it saw before sending, which makes a
 * stale acknowledgment from a previous command impossible to accept.
 */
struct ToolState {
  // Protects @c ack and @c sequence.
  std::mutex mutex;

  // Wakes the waiting worker when an acknowledgment arrives.
  std::condition_variable wake;

  // Last acknowledgment received on the tool channel.
  proto::ToolAcknowledgment ack;

  // Monotonic counter incremented for every stored acknowledgment.
  uint64_t sequence{0};
};
}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_TOOL_STATE_HPP_

// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file tool_state.hpp
 * @brief Correlate tool command acknowledgments. */
#pragma once
#include <condition_variable>
#include <mutex>

#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/** @brief Shared acknowledgment state retained by native reader callbacks. */
struct ToolState {
  std::mutex mutex;
  std::condition_variable wake;
  proto::ToolAcknowledgment ack;
  uint64_t sequence{0};
};
}  // namespace autonomy::task::coverage

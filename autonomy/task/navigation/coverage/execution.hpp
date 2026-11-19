// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file execution.hpp
 * @brief Execution coverage execution contract. */
#pragma once
#include "autonomy/task/navigation/coverage/coverage_server.hpp"
#include "behaviortree_cpp/basic_types.h"
namespace autonomy::task::coverage {
/** @brief Execution execution lifecycle and task actions. */
class Execution {
 public:
  virtual ~Execution() = default;
  /** @brief Execute the requested coverage planning pipeline. */
  virtual BT::NodeStatus Compute() = 0;
  /** @brief Reach the first coverage pose through the native navigation
   * planner. */
  virtual BT::NodeStatus NavigateToStart() = 0;
  /** @brief Advance only after the current coverage segment terminates
   * successfully. */
  virtual BT::NodeStatus FollowCoverage() = 0;
  /** @brief Revoke further dispatch and request goal-specific cancellation. */
  virtual void Cancel() = 0;
};
}  // namespace autonomy::task::coverage

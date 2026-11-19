// SPDX-License-Identifier: Apache-2.0
/** @file fake_execution.hpp
 * @brief Deterministic execution seam for coverage behavior-tree checks. */
#pragma once
#include "autonomy/task/navigation/coverage/execution.hpp"
namespace autonomy::task::coverage {
/** @brief Record behavior-tree stage dispatch and cancellation. */
struct FakeExecution : Execution {
  int compute{0}, approach{0}, follow{0}, cancel{0};
  bool fail{false};
  /** @brief Execute the requested coverage planning pipeline. */
  BT::NodeStatus Compute() override {
    ++compute;
    return fail ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
  /** @brief Reach the first coverage pose through the native navigation
   * planner. */
  BT::NodeStatus NavigateToStart() override {
    ++approach;
    return BT::NodeStatus::SUCCESS;
  }
  /** @brief Advance only after the current coverage segment terminates
   * successfully. */
  BT::NodeStatus FollowCoverage() override {
    ++follow;
    return follow < 3 ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
  }
  /** @brief Revoke further dispatch and request goal-specific cancellation. */
  void Cancel() override { ++cancel; }
};
}  // namespace autonomy::task::coverage

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
 * @file fake_execution.hpp
 * @brief Deterministic execution seam for coverage behavior-tree checks.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_TESTS_FAKE_EXECUTION_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_TESTS_FAKE_EXECUTION_HPP_

#include "autonomy/task/navigation/coverage/execution.hpp"
namespace autonomy::task::coverage {
/**
 * @brief Record behavior-tree stage dispatch and cancellation.
 */
struct FakeExecution : Execution {
  int compute{0}, approach{0}, follow{0}, cancel{0};
  bool fail{false};
  /**
   * @brief Execute the requested coverage planning pipeline.
   */
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
  /**
   * @brief Revoke further dispatch and request goal-specific cancellation.
   */
  void Cancel() override { ++cancel; }
};
}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_TESTS_FAKE_EXECUTION_HPP_

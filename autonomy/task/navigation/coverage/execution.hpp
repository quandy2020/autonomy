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
 * @file execution.hpp
 * @brief Execution coverage execution contract.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_EXECUTION_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_EXECUTION_HPP_

#include "autonomy/task/navigation/coverage/coverage_server.hpp"
#include "behaviortree_cpp/basic_types.h"
namespace autonomy::task::coverage {
/**
 * @class autonomy::task::coverage::Execution
 * @brief Lifecycle hooks the coverage behavior tree drives.
 *
 * The tree itself only sequences the three steps; the concrete execution owns
 * the planner client, the navigator client and the tool, and reports each step
 * as a BT status so a running step keeps the tick loop alive.
 */
class Execution {
 public:
  virtual ~Execution() = default;

  /**
   * @brief Plans the coverage path for the current goal.
   * @return RUNNING while the planner is busy, SUCCESS once a path is stored,
   *         FAILURE when the plan is rejected.
   */
  virtual BT::NodeStatus Compute() = 0;

  /**
   * @brief Drives the robot to the first coverage pose.
   * @return RUNNING while the approach runs, SUCCESS on arrival (also when the
   *         robot is already there), FAILURE when planning the approach fails.
   */
  virtual BT::NodeStatus NavigateToStart() = 0;

  /**
   * @brief Executes the planned coverage, one bounded segment at a time.
   * @return RUNNING until the last segment finishes, then SUCCESS; FAILURE when
   *         the follower or the tool reports an error.
   */
  virtual BT::NodeStatus FollowCoverage() = 0;

  /**
   * @brief Revokes further dispatch and requests goal-specific cancellation.
   */
  virtual void Cancel() = 0;
};
}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_EXECUTION_HPP_

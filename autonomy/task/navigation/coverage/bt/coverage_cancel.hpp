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
 * @file coverage_cancel.hpp
 * @brief Native coverage behavior-tree action.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_BT_COVERAGE_CANCEL_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_BT_COVERAGE_CANCEL_HPP_

#include "autonomy/task/navigation/coverage/bt/coverage_nodes.hpp"
#include "autonomy/task/navigation/coverage/native_execution.hpp"
namespace autonomy::task::coverage {
/**
 * @class autonomy::task::coverage::CoverageCancel
 * @brief Cancel node used by the recovery branch of the coverage tree.
 *
 * Cancels the running execution and always succeeds, so the tree can continue
 * with its failure handling instead of stopping on the cancel node itself.
 */
class CoverageCancel : public BT::SyncActionNode {
 public:
  using BT::SyncActionNode::SyncActionNode;

  /**
   * @brief The cancel node takes no ports.
   * @return Empty port list.
   */
  static BT::PortsList providedPorts() { return {}; }

  /**
   * @brief Cancels the execution bound to the blackboard.
   * @return Always SUCCESS.
   */
  BT::NodeStatus tick() override {
    config()
        .blackboard->get<std::shared_ptr<Execution>>(kExecutionBlackboard)
        ->Cancel();
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_BT_COVERAGE_CANCEL_HPP_

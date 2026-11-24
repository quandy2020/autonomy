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
 * @file coverage_nodes.hpp
 * @brief Native complete-coverage coverage nodes implementation.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_BT_COVERAGE_NODES_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_BT_COVERAGE_NODES_HPP_

#include "autonomy/task/navigation/coverage/native_execution.hpp"
#include "autonomy/task/navigation/coverage/path_segments.hpp"
#include "behaviortree_cpp/bt_factory.h"
namespace autonomy::task::coverage {
// Blackboard key holding the std::shared_ptr<Execution> shared by every node.
inline constexpr char kExecutionBlackboard[] = "coverage_execution";

/**
 * @brief Registers the coverage nodes with a behavior tree factory.
 *
 * Registers the three stage nodes (compute, approach, follow), the cancel
 * action and the retry guard, so a tree XML can reference them by name.
 *
 * @param[in] factory Factory that will build the coverage tree.
 */
void RegisterCoverageNodes(BT::BehaviorTreeFactory& factory);
}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_BT_COVERAGE_NODES_HPP_

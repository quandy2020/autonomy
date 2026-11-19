// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file coverage_nodes.hpp
 * @brief Native complete-coverage coverage nodes implementation. */
#pragma once
#include "autonomy/task/navigation/coverage/native_execution.hpp"
#include "autonomy/task/navigation/coverage/path_segments.hpp"
#include "behaviortree_cpp/bt_factory.h"
namespace autonomy::task::coverage {
inline constexpr char kExecutionBlackboard[] = "coverage_execution";
void RegisterCoverageNodes(BT::BehaviorTreeFactory&);
}  // namespace autonomy::task::coverage

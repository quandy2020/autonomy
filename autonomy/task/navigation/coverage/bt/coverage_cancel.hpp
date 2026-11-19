// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file coverage_cancel.hpp
 * @brief Native coverage behavior-tree action. */
#pragma once
#include "autonomy/task/navigation/coverage/bt/coverage_nodes.hpp"
#include "autonomy/task/navigation/coverage/native_execution.hpp"
namespace autonomy::task::coverage {
class CoverageCancel : public BT::SyncActionNode {
 public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    config()
        .blackboard->get<std::shared_ptr<Execution>>(kExecutionBlackboard)
        ->Cancel();
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace autonomy::task::coverage

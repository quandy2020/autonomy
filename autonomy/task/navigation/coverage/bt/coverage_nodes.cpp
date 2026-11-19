// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file coverage_nodes.cpp
 * @brief Native complete-coverage coverage nodes implementation. */
#include "autonomy/task/navigation/coverage/bt/coverage_cancel.hpp"
#include "autonomy/task/navigation/coverage/bt/coverage_node.hpp"
#include "autonomy/task/behavior_tree/bt_node_registry.hpp"
namespace autonomy::task::coverage {
void RegisterCoverageNodes(BT::BehaviorTreeFactory& factory) {
  factory.registerNodeType<CoverageNode<0>>("ComputeCoveragePath");
  factory.registerNodeType<CoverageNode<1>>("NavigateToCoverageStart");
  factory.registerNodeType<CoverageNode<2>>("FollowCoveragePath");
  factory.registerNodeType<CoverageCancel>("CoverageCancel");
}

namespace {
// Register with ordinary BtRunner factories without declaring another class.
const bool kCoverageNodesRegistered = [] {
  ::autonomy::task::AddBtNodeRegistrar(&RegisterCoverageNodes);
  return true;
}();
}  // namespace

}  // namespace autonomy::task::coverage

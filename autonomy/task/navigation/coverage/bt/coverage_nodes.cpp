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
 * @file coverage_nodes.cpp
 * @brief Native complete-coverage coverage nodes implementation.
 */

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

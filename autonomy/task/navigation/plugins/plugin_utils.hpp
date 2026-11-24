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
 * @file plugin_utils.hpp
 * @brief Resolve native navigation clients and invocation-owned dispatch gates.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_PLUGINS_PLUGIN_UTILS_HPP_
#define AUTONOMY_TASK_NAVIGATION_PLUGINS_PLUGIN_UTILS_HPP_

#include "autonomy/task/behavior_tree/plugin_utils.hpp"
#include "autonomy/task/navigation/navigation_client.hpp"
#include "autonomy/task/common/motion_dispatch_gate.hpp"

namespace autonomy::task::plugins::navigation {

/**
 * @brief Resolve the invocation gate without silently bypassing native ownership.
 * @return Gate, or null only for an explicitly legacy non-NavigationTask tree.
 * @throws std::runtime_error When a native invocation has no valid gate.
 */
inline std::shared_ptr<common::MotionDispatchGate> ResolveMotionGate(const BT::TreeNode& node) {
    std::shared_ptr<common::MotionDispatchGate> gate;
    const auto board = node.config().blackboard;
    if (board) board->get(common::kMotionDispatchGateBlackboardKey, gate);
    uint64_t epoch = 0;
    if (!gate && board && board->get("navigation_invocation_epoch", epoch) && epoch != 0)
        throw std::runtime_error("native navigation invocation has no motion dispatch gate");
    return gate;
}

/**
 * @brief Resolves the navigation client owned by this invocation.
 * @param[in] node Behavior tree node resolving the client.
 * @return Shared client; never null for a native navigation invocation.
 * @throws std::runtime_error when the invocation has no installed client.
 */
inline ::autonomy::task::navigation::NavigationClient::Ptr ResolveClient(
    const BT::TreeNode& node) {
    return ::autonomy::task::plugins::ResolveClient<
        ::autonomy::task::navigation::NavigationClient>(node);
}

using ::autonomy::task::plugins::SetErrorPorts;
using ::autonomy::task::plugins::ClearErrorPorts;

[[nodiscard]] inline automsgs::msgs::builtin_interfaces::Duration
ToProtoDuration(double seconds) {
    return ::autonomy::task::plugins::DurationFromSeconds(seconds);
}

}  // namespace autonomy::task::plugins::navigation

#endif  // AUTONOMY_TASK_NAVIGATION_PLUGINS_PLUGIN_UTILS_HPP_

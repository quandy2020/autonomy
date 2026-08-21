/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AUTONOMY_TASK_APPS_BEHAVIOR_TREE_PLUGIN_UTILS_HPP_
#define AUTONOMY_TASK_APPS_BEHAVIOR_TREE_PLUGIN_UTILS_HPP_

#include <string>
#include <type_traits>

#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include "behaviortree_cpp/tree_node.h"

namespace autonomy {
namespace task {
namespace plugins {

/**
 * Resolves a domain Client from the BT node blackboard (or process shared).
 * ClientType must provide static FromNode(const BT::TreeNode&).
 */
template <typename ClientType>
[[nodiscard]] inline typename ClientType::Ptr ResolveClient(
    const BT::TreeNode& node) {
    static_assert(
        std::is_same_v<decltype(ClientType::FromNode(node)),
                       typename ClientType::Ptr>,
        "ClientType must expose static FromNode -> Ptr");
    return ClientType::FromNode(node);
}

inline void SetErrorPorts(BT::TreeNode& node, int code,
                          const std::string& message) {
    node.setOutput("error_code_id", code);
    node.setOutput("error_msg", message);
}

inline void ClearErrorPorts(BT::TreeNode& node) {
    node.setOutput("error_code_id", 0);
    node.setOutput("error_msg", std::string{});
}

[[nodiscard]] inline automsgs::msgs::builtin_interfaces::Duration
DurationFromSeconds(double seconds) {
    return automsgs::msgs::builtin_interfaces::DurationFromSeconds(seconds);
}

}  // namespace plugins
}  // namespace task
}  // namespace autonomy

#endif  // AUTONOMY_TASK_APPS_BEHAVIOR_TREE_PLUGIN_UTILS_HPP_

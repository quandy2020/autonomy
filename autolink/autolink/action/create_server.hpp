/**
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#pragma once

#include <memory>
#include <string>

#include "autolink/action/server.hpp"
#include "autolink/node/node.hpp"

namespace autolink {
namespace action {

/**
 * @brief Create an action server.
 *
 * All provided callback functions must be non-blocking.
 *
 * @tparam ActionT The action type, which should have Goal, Feedback, and
 * Result types defined.
 * @param node The node to create the server on.
 * @param action_name The name of the action.
 * @param handle_goal A callback that decides if a goal should be accepted or
 * rejected.
 * @param handle_cancel A callback that decides if a goal should be attempted
 * to be canceled.
 * @param handle_accepted A callback that is called to give the user a handle
 * to the goal.
 * @return std::shared_ptr<Server<ActionT>> A shared pointer to the created
 * server.
 */
template <typename ActionT>
std::shared_ptr<Server<ActionT>> CreateServer(
    std::shared_ptr<Node> node, const std::string& action_name,
    typename Server<ActionT>::GoalCallback handle_goal,
    typename Server<ActionT>::CancelCallback handle_cancel,
    typename Server<ActionT>::AcceptedCallback handle_accepted) {
    return std::make_shared<Server<ActionT>>(node, action_name, handle_goal,
                                             handle_cancel, handle_accepted);
}

}  // namespace action
}  // namespace autolink

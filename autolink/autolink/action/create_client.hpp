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

#include "autolink/action/client.hpp"
#include "autolink/node/node.hpp"

namespace autolink {
namespace action {

/**
 * @brief Create an action client.
 *
 * @tparam ActionT The action type, which should have Goal, Feedback, and
 * Result types defined.
 * @param node The node to create the client on.
 * @param action_name The name of the action.
 * @return std::shared_ptr<Client<ActionT>> A shared pointer to the created
 * client.
 */
template <typename ActionT>
std::shared_ptr<Client<ActionT>> CreateClient(std::shared_ptr<Node> node,
                                              const std::string& action_name) {
    return std::make_shared<Client<ActionT>>(node, action_name);
}

}  // namespace action
}  // namespace autolink

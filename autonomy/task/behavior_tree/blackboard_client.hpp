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

#ifndef AUTONOMY_TASK_APPS_BEHAVIOR_TREE_BLACKBOARD_CLIENT_HPP_
#define AUTONOMY_TASK_APPS_BEHAVIOR_TREE_BLACKBOARD_CLIENT_HPP_

#include <memory>

#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/tree_node.h"

namespace autonomy {
namespace task {

/**
 * Shared process-wide + blackboard lookup for task BT clients.
 *
 * Each domain Client keeps its own type identity; this helper removes the
 * duplicated SetShared / FromBlackboard / FromNode boilerplate.
 *
 * Usage:
 *   BlackboardClientStore<ChargingClient>::SetShared(client);
 *   auto client = BlackboardClientStore<ChargingClient>::FromNode(
 *       node, kChargingClientBlackboardKey);
 */
template <typename ClientType>
class BlackboardClientStore
{
public:
    using Pointer = std::shared_ptr<ClientType>;

    static void SetShared(const Pointer& client) {
        SharedWeak() = client;
    }

    [[nodiscard]] static Pointer FromBlackboard(
        const std::shared_ptr<BT::Blackboard>& blackboard, const char* key) {
        if (blackboard != nullptr && key != nullptr) {
            Pointer client;
            if (blackboard->get(key, client) && client) {
                return client;
            }
        }
        return SharedWeak().lock();
    }

    [[nodiscard]] static Pointer FromNode(const BT::TreeNode& node,
                                          const char* key) {
        return FromBlackboard(node.config().blackboard, key);
    }

private:
    static std::weak_ptr<ClientType>& SharedWeak() {
        static std::weak_ptr<ClientType> instance;
        return instance;
    }
};

}  // namespace task
}  // namespace autonomy

#endif  // AUTONOMY_TASK_APPS_BEHAVIOR_TREE_BLACKBOARD_CLIENT_HPP_

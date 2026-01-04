/*
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

#include "autonomy/tasks/behavior_tree/plugins/control/nonblocking_sequence.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace control {

NonblockingSequence::NonblockingSequence(const std::string& name)
    : BT::ControlNode(name, {}) {}

NonblockingSequence::NonblockingSequence(const std::string& name,
                                         const BT::NodeConfiguration& conf)
    : BT::ControlNode(name, conf) {}

BT::NodeStatus NonblockingSequence::tick() {
    bool all_success = true;

    for (std::size_t i = 0; i < children_nodes_.size(); ++i) {
        auto status = children_nodes_[i]->executeTick();
        switch (status) {
            case BT::NodeStatus::FAILURE:
                ControlNode::haltChildren();
                all_success = false;  // probably not needed
                return status;
            case BT::NodeStatus::SUCCESS:
                break;
            case BT::NodeStatus::RUNNING:
                all_success = false;
                break;
            default:
                std::stringstream error_msg;
                error_msg << "Invalid node status. Received status " << status
                          << "from child " << children_nodes_[i]->name();
                throw std::runtime_error(error_msg.str());
        }
    }

    // Wrap up.
    if (all_success) {
        ControlNode::haltChildren();
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

}  // namespace control
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<
        autonomy::tasks::behavior_tree::plugins::control::NonblockingSequence>(
        "NonblockingSequence");
}
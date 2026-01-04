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

#include "autonomy/tasks/behavior_tree/plugins/action/assisted_teleop_cancel_node.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

AssistedTeleopCancelNode::AssistedTeleopCancelNode(
    const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(xml_tag_name, conf) {}

BT::NodeStatus AssistedTeleopCancelNode::tick() {
    // TODO: Implement assisted teleop cancel behavior
    return BT::NodeStatus::SUCCESS;
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    BT::NodeBuilder builder = [](const std::string& name,
                                 const BT::NodeConfiguration& config) {
        return std::make_unique<autonomy::tasks::behavior_tree::plugins::
                                    action::AssistedTeleopCancelNode>(name,
                                                                      config);
    };

    factory.registerBuilder<autonomy::tasks::behavior_tree::plugins::action::
                                AssistedTeleopCancelNode>(
        "CancelAssistedTeleop", builder);
}

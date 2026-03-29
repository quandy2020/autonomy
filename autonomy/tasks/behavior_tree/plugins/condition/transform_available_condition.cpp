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

#include "autonomy/tasks/behavior_tree/plugins/condition/transform_available_condition.hpp"

#include "autolink/common/log.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

TransformAvailableCondition::TransformAvailableCondition(
    const std::string& condition_name, const BT::NodeConfiguration& conf)
    : BT::ConditionNode(condition_name, conf), was_found_(false) {
    node_ = config().blackboard->get<std::shared_ptr<::autolink::Node>>("node");
    tf_ =
        config().blackboard->get<std::shared_ptr<autonomy::transform::Buffer>>(
            "tf_buffer");
}

TransformAvailableCondition::~TransformAvailableCondition() {
    ADEBUG << "Shutting down TransformAvailableCondition BT node";
}

void TransformAvailableCondition::initialize() {
    getInput("child", child_frame_);
    getInput("parent", parent_frame_);

    if (child_frame_.empty() || parent_frame_.empty()) {
        AERROR << "Child frame (" << child_frame_ << ") or parent frame ("
               << parent_frame_ << ") were empty.";
        throw std::runtime_error(
            "TransformAvailableCondition: Child or parent frames not "
            "provided!");
    }

    ADEBUG << "Initialized an TransformAvailableCondition BT node";
}

BT::NodeStatus TransformAvailableCondition::tick() {
    if (!BT::isStatusActive(status())) {
        initialize();
    }

    if (was_found_) {
        return BT::NodeStatus::SUCCESS;
    }

    std::string tf_error;
    commsgs::builtin_interfaces::Time zero_time(0, 0);
    bool found = tf_->canTransform(parent_frame_, child_frame_, zero_time,
                                   0.01f, &tf_error);

    if (found) {
        was_found_ = true;
        return BT::NodeStatus::SUCCESS;
    }

    AINFO << "Transform from " << child_frame_ << " to " << parent_frame_
          << " was not found, tf error: " << tf_error;
    return BT::NodeStatus::FAILURE;
}

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::
                                 condition::TransformAvailableCondition>(
        "TransformAvailable");
}

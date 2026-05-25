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

#include "autonomy/tasks/behavior_tree/plugins/decorator/rate_controller.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace decorator {

RateController::RateController(const std::string& name,
                               const BT::NodeConfiguration& conf)
    : BT::DecoratorNode(name, conf), first_time_(false) {}

void RateController::initialize() {
    double hz = 1.0;
    getInput("hz", hz);
    period_ = 1.0 / hz;
}

BT::NodeStatus RateController::tick() {
    if (!BT::isStatusActive(status())) {
        initialize();
        start_ = std::chrono::high_resolution_clock::now();
        first_time_ = true;
    }

    setStatus(BT::NodeStatus::RUNNING);

    const auto now = std::chrono::high_resolution_clock::now();
    const auto elapsed = now - start_;
    using float_seconds = std::chrono::duration<double>;
    const auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

    if (first_time_ || child_node_->status() == BT::NodeStatus::RUNNING ||
        seconds.count() >= period_) {
        first_time_ = false;
        const BT::NodeStatus child_state = child_node_->executeTick();

        switch (child_state) {
            case BT::NodeStatus::SKIPPED:
            case BT::NodeStatus::RUNNING:
            case BT::NodeStatus::FAILURE:
                return child_state;
            case BT::NodeStatus::SUCCESS:
                start_ = std::chrono::high_resolution_clock::now();
                return BT::NodeStatus::SUCCESS;
            default:
                return BT::NodeStatus::FAILURE;
        }
    }

    return status();
}

}  // namespace decorator
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<
        autonomy::tasks::behavior_tree::plugins::decorator::RateController>(
        "RateController");
}

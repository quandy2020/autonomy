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

#include "autonomy/system/system.hpp"

#include "autonomy/tasks/task_manager.hpp"

namespace autonomy {
namespace system {

AutonomyNode::AutonomyNode(const proto::AutonomyOptions& options)
    : options_{options} {
    tf_buffer_ = TfBuffer::Instance();
    map_server_ = std::make_shared<map::MapServer>(options_.map_options());
    controller_server_ = std::make_shared<control::ControllerServer>(
        options_.controller_options());
    planner_server_ =
        std::make_shared<planning::PlannerServer>(options_.planner_options());

    tasks_ = std::make_shared<tasks::TaskManager>(options_.task_options());
}

void AutonomyNode::Start() {
    if (map_server_ != nullptr) {
        map_server_->Start();
    }

    if (controller_server_ != nullptr) {
        controller_server_->Start();
    }

    if (planner_server_ != nullptr) {
        planner_server_->Start();
    }

    if (tasks_ != nullptr) {
        tasks_->Start();
    }
}

void AutonomyNode::Shutdown() {
    if (map_server_ != nullptr) {
        map_server_->Shutdown();
    }

    if (controller_server_ != nullptr) {
        controller_server_->Shutdown();
    }

    if (planner_server_ != nullptr) {
        planner_server_->Shutdown();
    }

    if (tasks_ != nullptr) {
        tasks_->Shutdown();
    }
}

AutonomyNode::UniquePtr CreateAutonomy(const proto::AutonomyOptions& options) {
    return std::make_unique<AutonomyNode>(options);
}

}  // namespace system
}  // namespace autonomy
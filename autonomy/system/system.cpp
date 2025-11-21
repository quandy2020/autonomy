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

namespace autonomy {
namespace system { 

AutonomyNode::AutonomyNode(const proto::AutonomyOptions& options)
   : options_{options}
{
    tf_buffer_ = TfBuffer::Instance();
    map_server_ = std::make_shared<map::MapServer>(options_.map_options());
    controller_server_ = std::make_shared<control::ControllerServer>(options_.controller_options());
    planner_server_ = std::make_shared<planning::PlannerServer>(options_.planner_options(), tf_buffer_);
    visual_server_ = std::make_shared<visualization::VisualizationServer>(options_.visualization_options());
}

void AutonomyNode::Start()
{
    if (map_server_ != nullptr) {
        map_server_->Start();
    }

    if (controller_server_ != nullptr) {
        controller_server_->Start();
    }

    if (planner_server_ != nullptr) {
        planner_server_->Start();
    }

    if (visual_server_ != nullptr) {
        visual_server_->Start();
    }
}

void AutonomyNode::WaitForShutdown()
{
    if (map_server_ != nullptr) {
        map_server_->WaitForShutdown();
    }

    if (controller_server_ != nullptr) {
        controller_server_->WaitForShutdown();
    }

    if (planner_server_ != nullptr) {
        planner_server_->WaitForShutdown();
    }

    if (visual_server_ != nullptr) {
        visual_server_->WaitForShutdown();
    }
}

AutonomyNode::UniquePtr CreateAutonomy(const proto::AutonomyOptions& options)
{
    return std::make_unique<AutonomyNode>(options);
}

}   // namespace tasks
}   // namespace autonomy
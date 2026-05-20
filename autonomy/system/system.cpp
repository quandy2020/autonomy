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

#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy {
namespace system {
namespace {

void ApplyMapToCostmap(
    const map::costmap_2d::Costmap2DWrapper::SharedPtr& wrapper,
    const commsgs::map_msgs::OccupancyGrid::SharedPtr& map) {
    if (!wrapper || !map) {
        return;
    }
    if (!wrapper->applyOccupancyGrid(*map)) {
        AWARN << "Failed to apply map from MapServer to costmap wrapper";
    }
}

}  // namespace

AutonomyNode::AutonomyNode(const proto::AutonomyOptions& options)
    : options_{options} {
    tf_buffer_ = TfBuffer::Instance();
    map_server_ = std::make_shared<map::MapServer>(options_.map_options());
    controller_server_ = std::make_shared<control::ControllerServer>(
        options_.controller_options());
    planner_server_ =
        std::make_shared<planning::PlannerServer>(options_.planner_options());
    smoother_server_ = std::make_shared<planning::SmootherServer>(
        options_.planner_options(), planner_server_->GetCostmapWrapper());
    planner_server_->SetSmootherServer(smoother_server_);
}

AutonomyNode::~AutonomyNode() {
    Shutdown();
}

void AutonomyNode::Start() {
    task_context_ = std::make_shared<tasks::common::TaskContext>();
    task_context_->planner = planner_server_;
    task_context_->smoother = smoother_server_;
    task_context_->controller = controller_server_;
    task_context_->global_costmap = planner_server_
                                        ? planner_server_->GetCostmapWrapper()
                                        : nullptr;
    task_context_->local_costmap = controller_server_
                                       ? controller_server_->GetCostmapWrapper()
                                       : task_context_->global_costmap;
    task_context_->tf = std::shared_ptr<transform::Buffer>(
        tf_buffer_, [](transform::Buffer*) {});
    if (planner_server_) {
        const auto& planner_options = options_.planner_options();
        if (!planner_options.default_planner_id().empty()) {
            task_context_->selected_planner_id =
                planner_options.default_planner_id();
        } else {
            task_context_->selected_planner_id =
                planner_server_->GetDefaultPlannerId();
        }
        if (!planner_options.default_smoother_id().empty()) {
            task_context_->selected_smoother_id =
                planner_options.default_smoother_id();
        } else if (smoother_server_) {
            task_context_->selected_smoother_id =
                smoother_server_->GetDefaultSmootherId();
        }
        task_context_->global_frame =
            planner_options.costmap().frame_id().empty()
                ? "map"
                : planner_options.costmap().frame_id();
    }

    if (map_server_ != nullptr) {
        map_server_->SetMapPublishCallback(
            [this](const commsgs::map_msgs::OccupancyGrid::SharedPtr& map) {
                if (planner_server_) {
                    ApplyMapToCostmap(planner_server_->GetCostmapWrapper(),
                                      map);
                }
                if (controller_server_) {
                    ApplyMapToCostmap(controller_server_->GetCostmapWrapper(),
                                      map);
                }
            });
        map_server_->Start();
    }

    if (planner_server_ != nullptr) {
        planner_server_->Start();
    }

    if (smoother_server_ != nullptr) {
        smoother_server_->Start();
    }

    if (controller_server_ != nullptr) {
        const std::string global_frame =
            options_.planner_options().costmap().frame_id().empty()
                ? "map"
                : options_.planner_options().costmap().frame_id();
        std::shared_ptr<transform::Buffer> tf_shared(
            tf_buffer_, [](transform::Buffer*) {});
        controller_server_->SetNavigationContext(tf_shared, global_frame,
                                                 "base_link");
        if (planner_server_) {
            controller_server_->SetSharedCostmap(
                planner_server_->GetCostmapWrapper());
        }
        controller_server_->Start();
    }

    if (map_server_ != nullptr) {
        const auto map = map_server_->GetStaticMapShared();
        if (map) {
            if (planner_server_) {
                ApplyMapToCostmap(planner_server_->GetCostmapWrapper(), map);
            } else if (controller_server_) {
                ApplyMapToCostmap(controller_server_->GetCostmapWrapper(),
                                  map);
            }
        }
    }
}

void AutonomyNode::Shutdown() {
    if (controller_server_ != nullptr) {
        controller_server_->Shutdown();
    }

    if (smoother_server_ != nullptr) {
        smoother_server_->Shutdown();
    }

    if (planner_server_ != nullptr) {
        planner_server_->Shutdown();
    }

    if (map_server_ != nullptr) {
        map_server_->Shutdown();
    }
}

AutonomyNode::UniquePtr CreateAutonomy(const proto::AutonomyOptions& options) {
    return std::make_unique<AutonomyNode>(options);
}

}  // namespace system
}  // namespace autonomy

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

#include "autonomy/tasks/utils/costmap_clear_utils.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_layer.hpp"

namespace autonomy {
namespace tasks {
namespace utils {
namespace {

bool IsLocalCostmapService(const std::string& service_name) {
    return service_name.find("local_costmap") != std::string::npos;
}

void ClearPluginsInRegion(
    const map::costmap_2d::Costmap2DWrapper::SharedPtr& costmap, int start_x,
    int start_y, int end_x, int end_y, bool invert) {
    if (!costmap) {
        return;
    }
    auto* layered = costmap->getLayeredCostmap();
    if (!layered) {
        return;
    }
    auto* plugins = layered->getPlugins();
    if (!plugins) {
        return;
    }
    for (auto& plugin : *plugins) {
        auto* layer =
            dynamic_cast<map::costmap_2d::CostmapLayer*>(plugin.get());
        if (layer != nullptr) {
            layer->clearArea(start_x, start_y, end_x, end_y, invert);
        }
    }
}

bool RobotClearBounds(
    const map::costmap_2d::Costmap2DWrapper::SharedPtr& costmap,
    double reset_distance, int& start_x, int& start_y, int& end_x, int& end_y) {
    commsgs::geometry_msgs::PoseStamped pose;
    if (!costmap->getRobotPose(pose)) {
        AWARN << "ClearCostmap: robot pose unavailable.";
        return false;
    }

    map::costmap_2d::Costmap2D* master = costmap->getCostmap();
    if (!master) {
        return false;
    }

    unsigned int mx = 0;
    unsigned int my = 0;
    if (!master->worldToMap(pose.pose.position.x, pose.pose.position.y, mx,
                            my)) {
        AWARN << "ClearCostmap: robot pose outside costmap bounds.";
        return false;
    }

    const int dist_cells = static_cast<int>(
        std::ceil(reset_distance / master->getResolution()));
    start_x =
        std::max(0, static_cast<int>(mx) - dist_cells);
    start_y =
        std::max(0, static_cast<int>(my) - dist_cells);
    end_x = std::min(static_cast<int>(master->getSizeInCellsX()),
                     static_cast<int>(mx) + dist_cells);
    end_y = std::min(static_cast<int>(master->getSizeInCellsY()),
                     static_cast<int>(my) + dist_cells);
    return start_x < end_x && start_y < end_y;
}

}  // namespace

map::costmap_2d::Costmap2DWrapper::SharedPtr ResolveCostmap(
    const std::shared_ptr<common::TaskContext>& ctx,
    const std::string& service_name) {
    if (!ctx) {
        return nullptr;
    }
    if (IsLocalCostmapService(service_name) && ctx->controller) {
        return ctx->controller->GetCostmapWrapper();
    }
    if (ctx->planner) {
        return ctx->planner->GetCostmapWrapper();
    }
    if (ctx->controller) {
        return ctx->controller->GetCostmapWrapper();
    }
    return nullptr;
}

void ClearEntireCostmap(
    const map::costmap_2d::Costmap2DWrapper::SharedPtr& costmap) {
    if (!costmap) {
        AWARN << "ClearEntireCostmap: costmap is null.";
        return;
    }
    costmap->resetLayers();
}

void ClearCostmapAroundRobot(
    const map::costmap_2d::Costmap2DWrapper::SharedPtr& costmap,
    double reset_distance) {
    int start_x = 0;
    int start_y = 0;
    int end_x = 0;
    int end_y = 0;
    if (!RobotClearBounds(costmap, reset_distance, start_x, start_y, end_x,
                          end_y)) {
        return;
    }
    ClearPluginsInRegion(costmap, start_x, start_y, end_x, end_y, false);
}

void ClearCostmapExceptRegion(
    const map::costmap_2d::Costmap2DWrapper::SharedPtr& costmap,
    double reset_distance) {
    int start_x = 0;
    int start_y = 0;
    int end_x = 0;
    int end_y = 0;
    if (!RobotClearBounds(costmap, reset_distance, start_x, start_y, end_x,
                          end_y)) {
        return;
    }
    ClearPluginsInRegion(costmap, start_x, start_y, end_x, end_y, true);
}

}  // namespace utils
}  // namespace tasks
}  // namespace autonomy

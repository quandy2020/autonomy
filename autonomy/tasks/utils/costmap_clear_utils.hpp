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

#pragma once

#include <memory>
#include <string>

#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/tasks/common/task_context.hpp"

namespace autonomy {
namespace tasks {
namespace utils {

/** Resolve costmap from BT service_name (e.g. "global_costmap/clear_costmap"). */
map::costmap_2d::Costmap2DWrapper::SharedPtr ResolveCostmap(
    const std::shared_ptr<common::TaskContext>& ctx,
    const std::string& service_name);

void ClearEntireCostmap(
    const map::costmap_2d::Costmap2DWrapper::SharedPtr& costmap);

void ClearCostmapAroundRobot(
    const map::costmap_2d::Costmap2DWrapper::SharedPtr& costmap,
    double reset_distance);

void ClearCostmapExceptRegion(
    const map::costmap_2d::Costmap2DWrapper::SharedPtr& costmap,
    double reset_distance);

}  // namespace utils
}  // namespace tasks
}  // namespace autonomy

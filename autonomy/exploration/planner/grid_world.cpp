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

#include "autonomy/exploration/planner/grid_world.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/common/math/vec2d.hpp"

namespace autonomy {
namespace exploration {

GridWorld::GridWorld(const proto::ExplorationOptions& options)
{
    SetOptions(options);
}

void GridWorld::SetOptions(const proto::ExplorationOptions& options)
{
    options_ = options;
    cell_size_ = options.grid_world().cell_size();
    const double extent = options.grid_world().exploration_area_half_extent() * 2.0;
    cols_ = static_cast<int>(std::ceil(extent / cell_size_));
    rows_ = cols_;
    origin_x_ = -options.grid_world().exploration_area_half_extent();
    origin_y_ = -options.grid_world().exploration_area_half_extent();
    EnsureSize();
}

void GridWorld::SetOrigin(double x, double y)
{
    origin_x_ = x - cols_ * cell_size_ * 0.5;
    origin_y_ = y - rows_ * cell_size_ * 0.5;
}

void GridWorld::EnsureSize()
{
    const size_t n = static_cast<size_t>(cols_ * rows_);
    status_.assign(n, CellStatus::kUnseen);
    cell_gain_.assign(n, 0.0);
}

int GridWorld::CellIndex(double x, double y) const
{
    const int cx = static_cast<int>(std::floor((x - origin_x_) / cell_size_));
    const int cy = static_cast<int>(std::floor((y - origin_y_) / cell_size_));
    if (cx < 0 || cy < 0 || cx >= cols_ || cy >= rows_) {
        return -1;
    }
    return cy * cols_ + cx;
}

commsgs::geometry_msgs::Point GridWorld::CellCenter(int cell_index) const
{
    commsgs::geometry_msgs::Point p;
    if (cell_index < 0 || static_cast<size_t>(cell_index) >= status_.size()) {
        return p;
    }
    const int cx = cell_index % cols_;
    const int cy = cell_index / cols_;
    p.x = origin_x_ + (static_cast<double>(cx) + 0.5) * cell_size_;
    p.y = origin_y_ + (static_cast<double>(cy) + 0.5) * cell_size_;
    p.z = 0.0;
    return p;
}

bool GridWorld::IsNearbyCell(int cell_index, double robot_x, double robot_y,
                             int nearby_radius) const
{
    if (cell_index < 0) {
        return false;
    }
    const int cx = cell_index % cols_;
    const int cy = cell_index / cols_;
    const int rcx = static_cast<int>(
        std::floor((robot_x - origin_x_) / cell_size_));
    const int rcy = static_cast<int>(
        std::floor((robot_y - origin_y_) / cell_size_));
    return std::abs(cx - rcx) <= nearby_radius &&
           std::abs(cy - rcy) <= nearby_radius;
}

void GridWorld::UpdateCellStatus(const ViewpointManager& viewpoints,
                                 const PlanningEnv& env)
{
    std::fill(cell_gain_.begin(), cell_gain_.end(), 0.0);
    for (const auto& vp : viewpoints.viewpoints()) {
        if (vp.cell_index < 0 ||
            static_cast<size_t>(vp.cell_index) >= cell_gain_.size()) {
            continue;
        }
        cell_gain_[static_cast<size_t>(vp.cell_index)] =
            std::max(cell_gain_[static_cast<size_t>(vp.cell_index)], vp.gain);
    }

    for (size_t i = 0; i < status_.size(); ++i) {
        const double gain = cell_gain_[i];
        auto& st = status_[i];
        const auto center = CellCenter(static_cast<int>(i));
        if (!env.IsInExplorationArea(center.x, center.y)) {
            continue;
        }
        if (gain >= options_.viewpoint().min_gain()) {
            st = CellStatus::kExploring;
        } else if (st == CellStatus::kExploring &&
                   gain < options_.grid_world().covered_gain_threshold()) {
            st = CellStatus::kCovered;
        } else if (st == CellStatus::kUnseen && gain > 0.0) {
            st = CellStatus::kExploring;
        }
    }
}

std::vector<int> GridWorld::GetExploringCells(
    double robot_x, double robot_y, int nearby_radius) const
{
    std::vector<int> cells;
    for (size_t i = 0; i < status_.size(); ++i) {
        if (status_[i] != CellStatus::kExploring) {
            continue;
        }
        if (IsNearbyCell(static_cast<int>(i), robot_x, robot_y,
                         nearby_radius)) {
            continue;
        }
        cells.push_back(static_cast<int>(i));
    }
    return cells;
}

float GridWorld::CoverageProgress() const
{
    if (status_.empty()) {
        return 0.f;
    }
    size_t covered = 0;
    for (const auto st : status_) {
        if (st == CellStatus::kCovered) {
            ++covered;
        }
    }
    return static_cast<float>(covered) /
           static_cast<float>(status_.size());
}

}  // namespace exploration
}  // namespace autonomy

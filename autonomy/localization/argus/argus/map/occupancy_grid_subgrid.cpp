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

#include "argus/map/occupancy_grid_subgrid.hpp"

#include <cstring>

namespace argus {
namespace map {

OccupancyGridSubgrid::OccupancyGridSubgrid() = default;

OccupancyGridSubgrid::~OccupancyGridSubgrid() {
    delete[] cells_;
    cells_ = nullptr;
}

OccupancyGridSubgrid::OccupancyGridSubgrid(const OccupancyGridSubgrid& other) {
    cells_ = nullptr;
    std::lock_guard<std::mutex> lock(other.mutex_);
    if (other.cells_ != nullptr) {
        MallocGrid();
        std::memcpy(cells_, other.cells_, sizeof(OccupancyGridCell) * kWidth2);
    }
}

OccupancyGridSubgrid& OccupancyGridSubgrid::operator=(
    const OccupancyGridSubgrid& other) {
    if (this == &other) {
        return *this;
    }
    std::scoped_lock locks(mutex_, other.mutex_);
    delete[] cells_;
    cells_ = nullptr;
    if (other.cells_ != nullptr) {
        MallocGrid();
        std::memcpy(cells_, other.cells_, sizeof(OccupancyGridCell) * kWidth2);
    }
    return *this;
}

void OccupancyGridSubgrid::MallocGrid() {
    if (cells_ != nullptr) {
        return;
    }
    cells_ = new OccupancyGridCell[kWidth2];
}

void OccupancyGridSubgrid::SetHitUnlocked(bool hit, int sub_xi, int sub_yi,
                                          float height) {
    if (cells_ == nullptr) {
        MallocGrid();
    }
    OccupancyGridCell& cell = cells_[sub_xi + sub_yi * kWidth];
    if (hit) {
        if (height < cell.height) {
            cell.height = height;
            cell.hit_cnt += 1;
            cell.visit_cnt += 1;
        }
    } else {
        if (cell.hit_cnt > 3) {
            if (height < cell.height) {
                cell.visit_cnt += 1;
            }
        } else {
            cell.visit_cnt += 1;
            cell.height = height;
        }
    }
}

void OccupancyGridSubgrid::SetHit(bool hit, int sub_xi, int sub_yi,
                                  float height) {
    std::lock_guard<std::mutex> lock(mutex_);
    SetHitUnlocked(hit, sub_xi, sub_yi, height);
}

void OccupancyGridSubgrid::GetHitAndVisit(int sx, int sy, unsigned int* hit_cnt,
                                          unsigned int* visit_cnt) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (cells_ == nullptr || hit_cnt == nullptr || visit_cnt == nullptr) {
        if (hit_cnt != nullptr) {
            *hit_cnt = 0;
        }
        if (visit_cnt != nullptr) {
            *visit_cnt = 0;
        }
        return;
    }
    const OccupancyGridCell& d = cells_[sx + sy * kWidth];
    *hit_cnt = d.hit_cnt;
    *visit_cnt = d.visit_cnt;
}

bool OccupancyGridSubgrid::empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cells_ == nullptr;
}

}  // namespace map
}  // namespace argus

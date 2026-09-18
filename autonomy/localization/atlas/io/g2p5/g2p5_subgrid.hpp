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

#pragma once

#include "autonomy/localization/atlas/io/g2p5/g2p5_grid_data.hpp"

#include <mutex>

namespace autonomy::localization::atlas {
namespace map {

//! Lazy-allocated square sub-grid (side = 1 << SUB_GRID_SIZE).
class SubGrid {
public:
    static inline constexpr int SUB_GRID_SIZE = 4;

    SubGrid(int num_x = 0, int num_y = 0);
    ~SubGrid();

    SubGrid(const SubGrid& other);
    SubGrid& operator=(const SubGrid& other);

    void SetGridHitPoint(bool hit, int sub_xi, int sub_yi, float height) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (grid_data_ == nullptr) {
            MallocGrid();
        }

        const int index = sub_xi + sub_yi * width_;
        GridData& cell = grid_data_[index];

        if (hit) {
            if (height < cell.height_) {
                cell.height_ = height;
                cell.hit_cnt_ += 1;
                cell.visit_cnt_ += 1;
            }
        } else {
            if (cell.hit_cnt_ > 3) {
                if (height < cell.height_) {
                    cell.visit_cnt_ += 1;
                }
            } else {
                cell.visit_cnt_ += 1;
                cell.height_ = height;
            }
        }
    }

    void RemoveCarNoise(int sub_xi, int sub_yi) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (grid_data_ == nullptr) {
            MallocGrid();
        }
        const int index = sub_xi + sub_yi * width_;
        grid_data_[index].visit_cnt_ += 4;
        grid_data_[index].hit_cnt_ = 0;
    }

    bool IsEmpty() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return grid_data_ == nullptr;
    }

    void GetHitAndVisit(int sx, int sy, unsigned int& hit_cnt,
                        unsigned int& visit_cnt) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        GridData& d = grid_data_[sx + sy * width_];
        hit_cnt = d.hit_cnt_;
        visit_cnt = d.visit_cnt_;
    }

private:
    void MallocGrid();

    static inline constexpr int width_ = (1 << SUB_GRID_SIZE);
    static inline constexpr int width_2_ =
        (1 << SUB_GRID_SIZE) * (1 << SUB_GRID_SIZE);

    std::mutex data_mutex_;
    GridData* grid_data_ = nullptr;
};

}  // namespace map
}  // namespace autonomy::localization::atlas

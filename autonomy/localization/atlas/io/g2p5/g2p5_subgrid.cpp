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

#include "autonomy/localization/atlas/io/g2p5/g2p5_subgrid.hpp"

#include <cstring>

namespace autonomy::localization::atlas {
namespace map {

SubGrid::SubGrid(int num_x, int num_y) {
    if (num_x * num_y == 0) {
        grid_data_ = nullptr;
        return;
    }
    grid_data_ = new GridData[static_cast<std::size_t>(num_x * num_y)];
}

void SubGrid::MallocGrid() {
    if (grid_data_ != nullptr) {
        return;
    }
    grid_data_ = new GridData[width_2_];
}

SubGrid::~SubGrid() {
    delete[] grid_data_;
    grid_data_ = nullptr;
}

SubGrid::SubGrid(const SubGrid& other) {
    grid_data_ = nullptr;
    if (other.grid_data_ != nullptr) {
        MallocGrid();
        std::memcpy(grid_data_, other.grid_data_,
                    sizeof(GridData) * width_2_);
    }
}

SubGrid& SubGrid::operator=(const SubGrid& other) {
    if (this != &other) {
        delete[] grid_data_;
        grid_data_ = nullptr;
        if (other.grid_data_ != nullptr) {
            MallocGrid();
            std::memcpy(grid_data_, other.grid_data_,
                        sizeof(GridData) * width_2_);
        }
    }
    return *this;
}

}  // namespace map
}  // namespace autonomy::localization::atlas

/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include <algorithm>
#include <cstdint>
#include <vector>

namespace autonomy {
namespace map {
namespace costmap_2d {

/**
 * @brief Compact 3D voxel column storage (up to 16 Z slices per XY cell).
 *
 * Each Z level uses 2 bits in the column word (Nav2-compatible layout):
 * bit (2*z) = marked, bit (2*z+1) = unknown.
 */
class VoxelGrid
{
public:
    static constexpr unsigned int VOXEL_BITS = 16;

    VoxelGrid() = default;

    void resize(unsigned int size_x, unsigned int size_y, unsigned int size_z) {
        size_x_ = size_x;
        size_y_ = size_y;
        size_z_ = std::min(size_z, VOXEL_BITS);
        data_.assign(size_x_ * size_y_, 0u);
    }

    void reset() { std::fill(data_.begin(), data_.end(), 0u); }

    unsigned int sizeX() const { return size_x_; }
    unsigned int sizeY() const { return size_y_; }
    unsigned int sizeZ() const { return size_z_; }

    unsigned int* getData() { return data_.data(); }
    const unsigned int* getData() const { return data_.data(); }

    void clearVoxelColumn(unsigned int mx, unsigned int my) {
        if (mx < size_x_ && my < size_y_) {
            data_[my * size_x_ + mx] = 0u;
        }
    }

    void markVoxel(unsigned int mx, unsigned int my, unsigned int mz) {
        if (mx >= size_x_ || my >= size_y_ || mz >= size_z_) {
            return;
        }
        data_[my * size_x_ + mx] |= (1u << (2u * mz));
    }

    unsigned int countMarkedVoxels(unsigned int mx, unsigned int my) const {
        if (mx >= size_x_ || my >= size_y_) {
            return 0;
        }
        const unsigned int column = data_[my * size_x_ + mx];
        unsigned int count = 0;
        for (unsigned int z = 0; z < size_z_; ++z) {
            if (column & (1u << (2u * z))) {
                ++count;
            }
        }
        return count;
    }

    bool markVoxelInMap(unsigned int mx, unsigned int my, unsigned int mz,
                        unsigned int mark_threshold) {
        markVoxel(mx, my, mz);
        return countMarkedVoxels(mx, my) >= mark_threshold;
    }

private:
    std::vector<unsigned int> data_;
    unsigned int size_x_{0};
    unsigned int size_y_{0};
    unsigned int size_z_{0};
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy

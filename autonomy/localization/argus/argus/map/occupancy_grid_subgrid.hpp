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

/**
 * @file occupancy_grid_subgrid.hpp
 * @brief Lazy 16×16 fine cell block (atlas g2p5 SubGrid).
 */

#ifndef ARGUS_MAP_OCCUPANCY_GRID_SUBGRID_HPP_
#define ARGUS_MAP_OCCUPANCY_GRID_SUBGRID_HPP_

#include <mutex>

#include "argus/map/occupancy_grid_data.hpp"

namespace argus {
namespace map {

/**
 * @brief Coarse tile holding a lazily allocated fine occupancy grid.
 *
 * Fine resolution cells are `kWidth × kWidth` with `kWidth = 1 << kLog2Width`.
 */
class OccupancyGridSubgrid
{
public:
    /** @brief log2 of fine grid edge length (16 cells). */
    static constexpr int kLog2Width = 4;
    /** @brief Fine cells along one edge. */
    static constexpr int kWidth = 1 << kLog2Width;

    OccupancyGridSubgrid();
    ~OccupancyGridSubgrid();

    OccupancyGridSubgrid(const OccupancyGridSubgrid& other);
    OccupancyGridSubgrid& operator=(const OccupancyGridSubgrid& other);

    /**
     * @brief Update one fine cell as hit or miss with height gating.
     * @param hit True if obstacle hit; false if free-space (miss) ray.
     * @param sub_xi Fine x index in [0, kWidth).
     * @param sub_yi Fine y index in [0, kWidth).
     * @param height Relative height used for overwrite rules (m).
     */
    void SetHit(bool hit, int sub_xi, int sub_yi, float height);

    /**
     * @brief Same as SetHit without locking (single-writer / Builder path).
     */
    void SetHitUnlocked(bool hit, int sub_xi, int sub_yi, float height);

    /**
     * @brief Read hit/visit counters for a fine cell.
     * @param sx Fine x.
     * @param sy Fine y.
     * @param[out] hit_cnt Hit count.
     * @param[out] visit_cnt Visit count.
     */
    void GetHitAndVisit(int sx, int sy, unsigned int* hit_cnt,
                        unsigned int* visit_cnt) const;

    /** @return True if fine storage has not been allocated. */
    bool empty() const;

private:
    void MallocGrid();

    static constexpr int kWidth2 = kWidth * kWidth;

    mutable std::mutex mutex_;
    OccupancyGridCell* cells_ = nullptr;
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_OCCUPANCY_GRID_SUBGRID_HPP_

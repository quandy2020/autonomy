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
 * @file occupancy_grid_map.hpp
 * @brief 2.5D hierarchical occupancy grid (atlas g2p5 G2P5Map).
 */

#ifndef ARGUS_MAP_OCCUPANCY_GRID_MAP_HPP_
#define ARGUS_MAP_OCCUPANCY_GRID_MAP_HPP_

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "argus/common/macros.hpp"
#include "argus/map/occupancy_grid_map_options.hpp"
#include "argus/map/occupancy_grid_subgrid.hpp"

namespace argus {
namespace map {

/**
 * @brief Hierarchical 2.5D occupancy map: coarse Subgrid tiles × fine cells.
 *
 * World XY bounds grow via Init/Resize. Ray hit/miss update fine cells with
 * height-aware free-space clearing (atlas g2p5 semantics).
 */
class OccupancyGridMap
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(OccupancyGridMap);

    static constexpr int kSubLog2 = OccupancyGridSubgrid::kLog2Width;
    static constexpr int kSubWidth = OccupancyGridSubgrid::kWidth;

    /**
     * @brief Construct with map options (does not allocate tiles until Init).
     * @param options Resolution and occupancy ratio.
     */
    explicit OccupancyGridMap(OccupancyGridMapOptions options = {});

    ~OccupancyGridMap();

    OccupancyGridMap(const OccupancyGridMap&) = delete;
    OccupancyGridMap& operator=(const OccupancyGridMap&) = delete;

    /** @brief Deep-copy tiles and bounds. */
    SharedPtr MakeDeepCopy() const;

    /**
     * @brief Allocate tiles covering axis-aligned world bounds.
     * @return false if size degenerate.
     */
    bool Init(float min_x, float min_y, float max_x, float max_y);

    /**
     * @brief Grow/shrink bounds, copying overlapping subgrids.
     * @return Always true after reallocation.
     */
    bool Resize(float min_x, float min_y, float max_x, float max_y);

    /** @brief Release all tiles. */
    void Clear();

    /**
     * @brief Mark a world XY sample as hit or free at height.
     * @param px World x (m).
     * @param py World y (m).
     * @param hit Obstacle hit when true.
     * @param height Relative height for gating (m).
     */
    void SetHitPoint(float px, float py, bool hit, float height);

    /**
     * @brief Same as SetHitPoint without per-cell locks (Builder single-thread).
     */
    void SetHitPointUnlocked(float px, float py, bool hit, float height);

    /**
     * @brief Rasterize free-space ray from LiDAR origin to endpoint.
     * @param point_x Endpoint world x.
     * @param point_y Endpoint world y.
     * @param origin_x Sensor origin world x.
     * @param origin_y Sensor origin world y.
     * @param height Endpoint relative height.
     * @param lidar_height Sensor height for ray height interpolation.
     */
    void SetMissPoint(float point_x, float point_y, float origin_x,
                      float origin_y, float height, float lidar_height);

    /**
     * @brief Same as SetMissPoint without per-cell locks (Builder single-thread).
     */
    void SetMissPointUnlocked(float point_x, float point_y, float origin_x,
                              float origin_y, float height, float lidar_height);

    /**
     * @brief Whether cell at world XY is occupied by occupancy_ratio.
     * @return false if OOB / unknown / free.
     */
    bool Occupied(float px, float py) const;

    /**
     * @brief Render BGR image: black=occ, white=free, gray=unknown (y-flipped).
     */
    cv::Mat ToCv() const;

    /** @return True if no tiles allocated. */
    bool empty() const {
        return grids_ == nullptr;
    }

    float resolution() const {
        return options_.resolution;
    }

    const OccupancyGridMapOptions& options() const {
        return options_;
    }

    void GetBounds(float* min_x, float* min_y, float* max_x,
                   float* max_y) const;

    int coarse_size_x() const {
        return grid_size_x_;
    }
    int coarse_size_y() const {
        return grid_size_y_;
    }

private:
    void UpdateCell(const Eigen::Vector2i& point_index, bool hit, float height);
    void UpdateCellUnlocked(const Eigen::Vector2i& point_index, bool hit,
                            float height);
    void SetMissPointImpl(float point_x, float point_y, float origin_x,
                          float origin_y, float height, float lidar_height,
                          bool unlocked);
    bool WorldToFineIndex(float x, float y, int* x_index, int* y_index) const;
    void ReleaseResources();

    float grid_reso_ = 0.f;  // coarse tile size in meters
    float min_x_ = 0.f;
    float min_y_ = 0.f;
    float max_x_ = 0.f;
    float max_y_ = 0.f;
    int grid_size_x_ = 0;
    int grid_size_y_ = 0;
    OccupancyGridMapOptions options_;
    OccupancyGridSubgrid** grids_ = nullptr;
    /** @brief Reused by SetMissPoint* (Builder / single-writer path). */
    std::vector<Eigen::Vector2i> miss_pts_buf_;
    std::vector<float> miss_h_buf_;
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_OCCUPANCY_GRID_MAP_HPP_

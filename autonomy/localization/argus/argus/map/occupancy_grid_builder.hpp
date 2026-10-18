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
 * @file occupancy_grid_builder.hpp
 * @brief Project LiDAR scans into a 2.5D OccupancyGridMap (atlas g2p5 rays).
 */

#ifndef ARGUS_MAP_OCCUPANCY_GRID_BUILDER_HPP_
#define ARGUS_MAP_OCCUPANCY_GRID_BUILDER_HPP_

#include "argus/common/macros.hpp"
#include "argus/common/math/transform/rigid_transform.hpp"
#include "argus/map/occupancy_grid_builder_options.hpp"
#include "argus/map/occupancy_grid_map.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace map {

/**
 * @brief Synchronous occupancy mapper: expand bounds and fuse body-frame scans.
 */
class OccupancyGridBuilder
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(OccupancyGridBuilder);

    /**
     * @brief Construct builder; map allocated on first AddScan.
     * @param options Floor band and map resolution.
     */
    explicit OccupancyGridBuilder(OccupancyGridBuilderOptions options = {});

    /**
     * @brief Fuse one scan (body cloud unless @p cloud_in_world).
     * @param T_w_b Body pose in world.
     * @param cloud_b Point cloud (body or world per flag).
     * @param cloud_in_world If true, @p cloud_b is already world-frame.
     */
    void AddScan(const transform::Rigid3d& T_w_b,
                 const sensor::PointCloudXYZI& cloud_b,
                 bool cloud_in_world = false);

    /** @return Shared occupancy map (may be null before first scan). */
    OccupancyGridMap::SharedPtr map() const {
        return map_;
    }

    /** @brief Drop map state. */
    void Reset();

    const OccupancyGridBuilderOptions& options() const {
        return options_;
    }

private:
    bool EnsureBounds(const transform::Rigid3d& T_w_b,
                      const sensor::PointCloudXYZI& cloud_b,
                      bool cloud_in_world);
    void ProjectScan(const transform::Rigid3d& T_w_b,
                     const sensor::PointCloudXYZI& cloud_b,
                     bool cloud_in_world);

    OccupancyGridBuilderOptions options_;
    OccupancyGridMap::SharedPtr map_;
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_OCCUPANCY_GRID_BUILDER_HPP_

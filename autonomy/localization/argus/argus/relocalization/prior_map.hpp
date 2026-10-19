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
 * @file prior_map.hpp
 * @brief Occupancy voxel prior for BBS scoring.
 */

#ifndef ARGUS_RELOCALIZATION_PRIOR_MAP_HPP_
#define ARGUS_RELOCALIZATION_PRIOR_MAP_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_set>

#include <Eigen/Core>

#include "argus/common/macros.hpp"
#include "argus/common/math/transform/rigid_transform.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace relocalization {

/** @brief Voxel-hashed occupancy map used as BnB / BBS score field. */
class PriorOccupancyMap
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(PriorOccupancyMap);

    /**
     * @brief Construct empty occupancy grid.
     * @param resolution_m Voxel edge length (m).
     */
    explicit PriorOccupancyMap(double resolution_m = 0.5);

    /** @brief Clear all occupied voxels and bounds. */
    void Clear();
    /**
     * @brief Mark voxels touched by a world-frame cloud.
     * @param cloud_w Points in world frame.
     */
    void InsertCloud(const sensor::PointCloudXYZI& cloud_w);
    /**
     * @brief Mark the voxel containing a world point.
     * @param p_w Point in world frame (m).
     */
    void InsertPoint(const Eigen::Vector3d& p_w);

    /**
     * @brief Test whether a world point lies in an occupied voxel.
     * @param p_w Query point in world frame.
     * @return true if the voxel is marked occupied.
     */
    bool Occupied(const Eigen::Vector3d& p_w) const;

    /**
     * @brief Fraction of query body-points that land in occupied voxels under
     * T.
     * @param cloud_b Body-frame scan used as BnB query.
     * @param T_w_b Candidate world pose of the body frame.
     * @return Hit ratio in [0, 1]; 0 if cloud empty.
     */
    double Score(const sensor::PointCloudXYZI& cloud_b,
                 const transform::Rigid3d& T_w_b) const;

    /** @return Number of occupied voxels. */
    std::size_t size() const {
        return occupied_.size();
    }
    /** @return Voxel resolution (m). */
    double resolution() const {
        return resolution_;
    }

    /** @return Axis-aligned min corner of inserted points (undefined if empty).
     */
    Eigen::Vector3d min_bound() const {
        return min_bound_;
    }
    /** @return Axis-aligned max corner of inserted points (undefined if empty).
     */
    Eigen::Vector3d max_bound() const {
        return max_bound_;
    }

    /**
     * @brief Reconstruct one point per occupied voxel (voxel centers).
     * @return World-frame cloud suitable for Scan Context place-index build.
     */
    sensor::PointCloudXYZI OccupiedCentersCloud() const;

private:
    int64_t MakeVoxelKey(int ix, int iy, int iz) const;
    void DecodeVoxelKey(int64_t key, int* ix, int* iy, int* iz) const;
    void VoxelIndex(const Eigen::Vector3d& p, int* ix, int* iy, int* iz) const;

    double resolution_ = 0.5;
    std::unordered_set<int64_t> occupied_;
    Eigen::Vector3d min_bound_ = Eigen::Vector3d::Constant(1e9);
    Eigen::Vector3d max_bound_ = Eigen::Vector3d::Constant(-1e9);
    bool has_bounds_ = false;
};

/**
 * @brief Occupancy prior plus the source world cloud (for SC++ place index).
 */
struct PriorMapBundle {
    PriorOccupancyMap::SharedPtr occupancy;
    sensor::PointCloudXYZI world_cloud;
    bool ok() const {
        return static_cast<bool>(occupancy) && occupancy->size() > 0;
    }
};

/** @brief Factory helpers to build PriorOccupancyMap from disk or clouds. */
class PriorMapLoader
{
public:
    /**
     * @brief Load ASCII xyz (one point per line: x y z [intensity]).
     * @param path Path to `.xyz` file.
     * @param resolution_m Voxel size for occupancy hashing.
     * @return Shared map, or nullptr on failure.
     */
    static PriorOccupancyMap::SharedPtr LoadXyz(const std::string& path,
                                                double resolution_m);

    /**
     * @brief Load `.xyz` and retain the world-frame point list.
     * @param path Path to `.xyz` file.
     * @param resolution_m Voxel size for occupancy hashing.
     * @return Bundle; `ok()` false on failure.
     */
    static PriorMapBundle LoadXyzBundle(const std::string& path,
                                        double resolution_m);

    /**
     * @brief Load from `.xyz` **or** map package dir (`prior.xyz` then
     * `map.xyz`).
     * @param path_or_dir File or map package directory.
     * @param resolution_m Voxel size when building occupancy.
     * @return Shared map, or nullptr on failure.
     */
    static PriorOccupancyMap::SharedPtr LoadPath(const std::string& path_or_dir,
                                                 double resolution_m);

    /**
     * @brief Like LoadPath but also returns the source world cloud.
     * @param path_or_dir File or map package directory.
     * @param resolution_m Voxel size when building occupancy.
     * @return Bundle; `ok()` false on failure.
     */
    static PriorMapBundle LoadPathBundle(const std::string& path_or_dir,
                                         double resolution_m);

    /**
     * @brief Build from an in-memory world-frame cloud.
     * @param cloud_w World-frame points to voxelize.
     * @param resolution_m Voxel edge length (m).
     * @return Shared occupancy map (never null on success).
     */
    static PriorOccupancyMap::SharedPtr FromCloud(
        const sensor::PointCloudXYZI& cloud_w, double resolution_m);
};

}  // namespace relocalization
}  // namespace argus

#endif  // ARGUS_RELOCALIZATION_PRIOR_MAP_HPP_

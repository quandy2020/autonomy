/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file esdf_map.hpp
 * @brief ROS-free FIESTA incremental ESDF on a dense 3D grid
 *        (IROS 2019, arXiv:1903.02144).
 */

#pragma once

#include <queue>
#include <vector>

#include <Eigen/Core>

#include "autonomy/map/esdf/options.hpp"

namespace autonomy {
namespace map {
namespace esdf {

/**
 * @brief Sentinel for missing voxel indices and unused list links.
 *
 * Stored in closest_obstacle_ components, obstacle_list_head_,
 * previous_list_index_, next_list_index_, and as the initial distance_buffer_
 * value before the first UpdateOccupancy on that cell.
 */
inline constexpr int kUndefined = -10000;

/**
 * @brief Large positive distance used for empty or newly allocated cells.
 *
 * Units match metres once multiplied by resolution in DistanceBetweenVoxels.
 * GetDistance maps any negative buffer value (including kUndefined) to this.
 */
inline constexpr int kInfinity = 10000;

/**
 * @class autonomy::map::esdf::EsdfMap
 * @brief Incremental Euclidean distance field on a fixed dense occupancy grid.
 *
 * Observe with SetOccupancy or IntegrateRay, then UpdateOccupancy + UpdateESDF.
 * Distance is metres to the nearest occupied voxel centre; occupied cells are 0.
 */
class EsdfMap {
public:
    /**
     * @brief Allocate occupancy and distance buffers from @p options.
     *
     * Copies geometry, computes grid_size_ / voxel_count_, initialises every
     * distance to kUndefined, and calls SetProbabilities with the option
     * probabilities. The BFS AABB is the full map (SetOriginalRange).
     *
     * @param[in] options Map origin, size, resolution, and occupancy fusion
     *                    probabilities. Copied; later edits to the argument
     *                    do not affect this instance.
     */
    explicit EsdfMap(Options options = {});

    /**
     * @brief Convert occupancy probabilities to cached log-odds clamps.
     *
     * Called from the constructor. Safe to call again to retune fusion without
     * reallocating buffers.
     *
     * @param[in] hit_probability Occupied-observation probability in (0, 1).
     *                            Becomes hit_log_odds_.
     * @param[in] miss_probability Free-observation probability in (0, 1).
     *                             Becomes miss_log_odds_.
     * @param[in] min_probability Lower occupancy-probability clamp in (0, 1).
     *                            Becomes min_log_odds_.
     * @param[in] max_probability Upper occupancy-probability clamp in (0, 1).
     *                            Becomes max_log_odds_.
     * @param[in] occupancy_probability Occupied if fused probability exceeds
     *                                  this. Becomes occupancy_log_odds_threshold_.
     */
    void SetProbabilities(double hit_probability, double miss_probability,
                          double min_probability, double max_probability,
                          double occupancy_probability);

    /**
     * @brief Count one hit or miss at a world position.
     *
     * Converts @p position to a voxel with floor((position - origin) /
     * resolution) and forwards to the voxel overload. Does not update
     * occupancy_buffer_ until UpdateOccupancy.
     *
     * @param[in] position World-frame sample in metres.
     * @param[in] occupied true to count a hit, false to count a miss. Every
     *                     call also increments observation_count_.
     * @return Dense linear voxel index, or kUndefined if @p position is
     *         outside [world_min_, world_max_].
     */
    int SetOccupancy(const Eigen::Vector3d& position, bool occupied);

    /**
     * @brief Count one hit or miss at a voxel index.
     *
     * Ignored (index still returned) when @p voxel is outside the current
     * update AABB. The first observation on a voxel pushes it onto
     * occupancy_queue_.
     *
     * @param[in] voxel Integer voxel coordinates (i, j, k), i along x.
     * @param[in] occupied true to count a hit, false to count a miss.
     * @return Dense linear voxel index from VoxelToIndex. Returned even when
     *         @p voxel is out of the update range.
     */
    int SetOccupancy(const Eigen::Vector3i& voxel, bool occupied);

    /**
     * @brief Occupancy class at a world position.
     *
     * @param[in] position World-frame query in metres.
     * @return 1 if occupancy_buffer_ exceeds occupancy_log_odds_threshold_,
     *         0 if free and inside the map, kUndefined if @p position is
     *         outside [world_min_, world_max_].
     */
    int GetOccupancy(const Eigen::Vector3d& position) const;

    /**
     * @brief Occupancy class at a voxel index.
     *
     * @param[in] voxel Integer voxel coordinates. Must be in-map; out-of-map
     *                  indices are not range-checked here.
     * @return 1 occupied, 0 free.
     */
    int GetOccupancy(const Eigen::Vector3i& voxel) const;

    /**
     * @brief Distance at a world position (nearest-voxel sample, not interpolated).
     *
     * @param[in] position World-frame query in metres.
     * @return Metres to the nearest occupied voxel centre. kInfinity if the
     *         voxel was never given a finite distance. kUndefined if
     *         @p position is outside the map.
     */
    double GetDistance(const Eigen::Vector3d& position) const;

    /**
     * @brief Distance at a voxel index (nearest-voxel sample).
     *
     * @param[in] voxel Integer voxel coordinates.
     * @return Metres to the nearest occupied voxel centre. Negative buffer
     *         values (unset) are reported as kInfinity.
     */
    double GetDistance(const Eigen::Vector3i& voxel) const;

    /**
     * @brief Trilinear distance and analytic gradient at a world point.
     *
     * Shifts @p position by half a voxel, samples the 2×2×2 neighbourhood, and
     * writes ∂distance/∂x,y,z in 1/metre. Occupied cells contribute 0.
     *
     * @param[in] position World-frame query in metres.
     * @param[out] gradient Distance gradient in 1/metre. Must not be null;
     *                      written only on success.
     * @return Interpolated distance in metres, or -1.0 if @p position is
     *         outside the map or @p gradient is null.
     */
    double GetDistanceWithGradient(const Eigen::Vector3d& position,
                                   Eigen::Vector3d* gradient) const;

    /**
     * @brief Whether occupancy observations are waiting to be fused.
     *
     * @return true when occupancy_queue_ is non-empty, so UpdateOccupancy
     *         has work.
     */
    bool CheckUpdate() const;

    /**
     * @brief Fuse hit/miss counts into log-odds and fill insert/delete queues.
     *
     * Each queued voxel gets one logit step (hit if hits >= misses, else miss),
     * then occupancy_buffer_ is clamped. Newly occupied voxels go to
     * insert_queue_; newly freed voxels go to delete_queue_.
     *
     * @param[in] reset_to_full_map When true, voxels are fused regardless of
     *            the previous update AABB. When false, voxels that left
     *            previous_update_* are reset to empty / kInfinity.
     * @return true when insert_queue_ or delete_queue_ is non-empty (call
     *         UpdateESDF next).
     */
    bool UpdateOccupancy(bool reset_to_full_map);

    /**
     * @brief Incremental 24-neighbour BFS from insert_queue_ and delete_queue_.
     *
     * Inserted occupied voxels become distance 0 and heads of their own
     * closest-obstacle lists. Deleted voxels rebuild neighbours from remaining
     * occupied seeds. Call after UpdateOccupancy when it returned true.
     */
    void UpdateESDF();

    /**
     * @brief Restrict BFS and OccupiedPoints to a world AABB.
     *
     * Corners are clamped to [world_min_, world_max_] and converted to voxel
     * indices. OccupiedPoints and VoxelInRange use this window until
     * SetOriginalRange.
     *
     * @param[in] min_position AABB minimum in metres (world frame).
     * @param[in] max_position AABB maximum in metres (world frame).
     * @param[in] remember_previous_range When true, copy the current window
     *            into previous_update_* before overwriting, so the next
     *            UpdateOccupancy(false) can reset voxels that left the window.
     */
    void SetUpdateRange(const Eigen::Vector3d& min_position,
                        const Eigen::Vector3d& max_position,
                        bool remember_previous_range = true);

    /**
     * @brief Restore the BFS AABB to the full allocated map.
     *
     * Sets update_min_voxel_ to (0,0,0) and update_max_voxel_ to
     * grid_size_ - 1, and copies the same into previous_update_*.
     */
    void SetOriginalRange();

    /**
     * @brief One FIESTA ray: endpoint occupied (or clamped free), free along
     *        the ray.
     *
     * @p origin and @p point are already in the map / world frame. Rays
     * shorter than min_ray_length are ignored. Rays longer than
     * max_ray_length are truncated and recorded as free only. Interior voxels
     * from Raycast (except the last) are misses.
     *
     * @param[in] origin Sensor origin in metres.
     * @param[in] point Hit (or far) point in metres.
     */
    void IntegrateRay(const Eigen::Vector3d& origin, const Eigen::Vector3d& point);

    /**
     * @brief IntegrateRay for each finite point; skips NaN / Inf coordinates.
     *
     * @param[in] origin Sensor origin in metres, shared by every ray.
     * @param[in] points Hit points in metres. A point is skipped when any
     *                   coordinate is not finite.
     */
    void IntegrateCloud(const Eigen::Vector3d& origin,
                        const std::vector<Eigen::Vector3d>& points);

    /**
     * @brief Occupied voxel centres inside the current update AABB.
     *
     * @return World-frame centres in metres for every voxel with
     *         occupancy_buffer_ above occupancy_log_odds_threshold_ in
     *         [update_min_voxel_, update_max_voxel_].
     */
    std::vector<Eigen::Vector3d> OccupiedPoints() const;

    /**
     * @brief Closest-obstacle doubly-linked-list self-check.
     *
     * Walks every allocated voxel and verifies previous/next links, and that
     * a cell with a finite distance and no previous link is the head of its
     * closest-obstacle list.
     *
     * @return true when every list link is consistent; false on the first
     *         broken link.
     */
    bool CheckConsistency() const;

    /**
     * @brief Number of voxels in the dense allocation.
     *
     * @return grid_size_.x() * y * z. The undefined-voxel list slot is
     *         voxel_count_ itself (not included in this count).
     */
    int voxel_count() const { return voxel_count_; }

    /**
     * @brief Geometry and occupancy-fusion parameters used at construction.
     *
     * @return Const reference to the copied Options. Probabilities may be
     *         stale if SetProbabilities was called afterwards; log-odds
     *         members reflect the latest call.
     */
    const Options& options() const { return options_; }

private:
    /**
     * @brief Occupancy / ESDF queue item: voxel and cached distance.
     */
    struct QueueElement {
        /**
         * @brief Integer voxel coordinates of the queued cell.
         */
        Eigen::Vector3i voxel{0, 0, 0};

        /**
         * @brief Distance in metres cached at enqueue time (0 for inserts).
         *
         * UpdateESDF skips an update_queue_ item when this no longer matches
         * distance_buffer_.
         */
        double distance{0.0};
    };

    /**
     * @brief Convert a probability in (0, 1) to log-odds.
     *
     * @param[in] probability Value in (0, 1); 0 and 1 are undefined for log.
     * @return log(probability / (1 - probability)).
     */
    static double ProbabilityToLogOdds(double probability);

    /**
     * @brief Occupied test on a dense linear index.
     *
     * @param[in] index Result of VoxelToIndex, in [0, voxel_count_).
     * @return true if occupancy_buffer_[index] >
     *         occupancy_log_odds_threshold_.
     */
    bool IsOccupied(int index) const;

    /**
     * @brief Whether a world point lies in the allocated AABB.
     *
     * @param[in] position World-frame point in metres.
     * @return true if each coordinate is in [world_min_, world_max_]
     *         inclusive.
     */
    bool PositionInMap(const Eigen::Vector3d& position) const;

    /**
     * @brief Whether a voxel lies in the current or previous BFS AABB.
     *
     * @param[in] voxel Integer voxel coordinates.
     * @param[in] use_current_range true to test [update_min_voxel_,
     *            update_max_voxel_]; false to test previous_update_*.
     * @return true if @p voxel is inside the chosen inclusive AABB.
     */
    bool VoxelInRange(const Eigen::Vector3i& voxel,
                      bool use_current_range = true) const;

    /**
     * @brief Convert a world point to a voxel index (floor).
     *
     * @param[in] position World-frame point in metres.
     * @param[out] voxel Integer (i, j, k). Must not be null.
     */
    void PositionToVoxel(const Eigen::Vector3d& position,
                         Eigen::Vector3i* voxel) const;

    /**
     * @brief Convert a voxel index to its world-frame centre.
     *
     * Centre is (voxel + 0.5) * resolution + origin.
     *
     * @param[in] voxel Integer voxel coordinates.
     * @param[out] position World-frame centre in metres. Must not be null.
     */
    void VoxelToPosition(const Eigen::Vector3i& voxel,
                         Eigen::Vector3d* position) const;

    /**
     * @brief Convert a voxel to a dense linear index.
     *
     * Layout is i * voxels_per_yz_plane_ + j * grid_size_.z() + k.
     *
     * @param[in] voxel Integer voxel coordinates. If voxel.x() is kUndefined,
     *                  returns undefined_voxel_index_ (the extra list slot).
     * @return Index in [0, voxel_count_) for in-map voxels, or
     *         undefined_voxel_index_ for the reserved undefined cell.
     */
    int VoxelToIndex(const Eigen::Vector3i& voxel) const;

    /**
     * @brief Inverse of VoxelToIndex for in-map indices.
     *
     * @param[in] index Dense linear index in [0, voxel_count_).
     * @return Integer voxel (i, j, k) such that VoxelToIndex(result) == index.
     */
    Eigen::Vector3i IndexToVoxel(int index) const;

    /**
     * @brief Euclidean distance between two voxel centres.
     *
     * @param[in] first First voxel index.
     * @param[in] second Second voxel index.
     * @return ||second - first|| * resolution, in metres.
     */
    double DistanceBetweenVoxels(const Eigen::Vector3i& first,
                                 const Eigen::Vector3i& second) const;

    /**
     * @brief Distance buffer value treating unset cells as kInfinity for BFS.
     *
     * @param[in] index Dense linear index in [0, voxel_count_).
     * @return distance_buffer_[index] if non-negative, otherwise kInfinity.
     */
    double StoredDistance(int index) const;

    /**
     * @brief Unlink a voxel from a closest-obstacle doubly linked list.
     *
     * @param[in] list_head_index Index of the occupied voxel whose list holds
     *            @p index (or undefined_voxel_index_ for unassigned cells).
     * @param[in] index Voxel to remove. Its previous/next links are set to
     *            kUndefined.
     */
    void DeleteFromList(int list_head_index, int index);

    /**
     * @brief Push a voxel onto the front of a closest-obstacle list.
     *
     * @param[in] list_head_index Occupied voxel that owns the list.
     * @param[in] index Voxel to insert as the new head.
     */
    void InsertIntoList(int list_head_index, int index);

    /**
     * @brief Copy of construction-time geometry and probability options.
     */
    Options options_;

    /**
     * @brief 1 / options_.resolution, cached for gradient scaling.
     */
    double inverse_resolution_{10.0};

    /**
     * @brief logit(hit_probability); added on an occupied observation.
     */
    double hit_log_odds_{0.0};

    /**
     * @brief logit(miss_probability); added on a free observation.
     */
    double miss_log_odds_{0.0};

    /**
     * @brief logit(min_probability); lower clamp of occupancy_buffer_.
     */
    double min_log_odds_{0.0};

    /**
     * @brief logit(max_probability); upper clamp of occupancy_buffer_.
     */
    double max_log_odds_{0.0};

    /**
     * @brief logit(occupancy_probability); occupied if occupancy_buffer_ exceeds this.
     */
    double occupancy_log_odds_threshold_{0.0};

    /**
     * @brief World-frame corner of voxel (0, 0, 0), metres. Same as options_.origin.
     */
    Eigen::Vector3d origin_{0, 0, 0};

    /**
     * @brief Inclusive world-frame AABB minimum of the allocated map, metres.
     */
    Eigen::Vector3d world_min_{0, 0, 0};

    /**
     * @brief Inclusive world-frame AABB maximum of the allocated map, metres.
     */
    Eigen::Vector3d world_max_{0, 0, 0};

    /**
     * @brief Number of voxels along x, y, z.
     */
    Eigen::Vector3i grid_size_{0, 0, 0};

    /**
     * @brief grid_size_.y() * grid_size_.z(); stride of the x index.
     */
    int voxels_per_yz_plane_{0};

    /**
     * @brief Total allocated voxels (grid_size_.x() * voxels_per_yz_plane_).
     */
    int voxel_count_{0};

    /**
     * @brief Extra list-head slot for cells whose closest obstacle is undefined.
     *
     * Equal to voxel_count_. obstacle_list_head_ is sized voxel_count_ + 1.
     */
    int undefined_voxel_index_{0};

    /**
     * @brief Inclusive minimum voxel of the current BFS / OccupiedPoints window.
     */
    Eigen::Vector3i update_min_voxel_{0, 0, 0};

    /**
     * @brief Inclusive maximum voxel of the current BFS / OccupiedPoints window.
     */
    Eigen::Vector3i update_max_voxel_{0, 0, 0};

    /**
     * @brief Previous update_min_voxel_, used when UpdateOccupancy(false).
     */
    Eigen::Vector3i previous_update_min_voxel_{0, 0, 0};

    /**
     * @brief Previous update_max_voxel_, used when UpdateOccupancy(false).
     */
    Eigen::Vector3i previous_update_max_voxel_{0, 0, 0};

    /**
     * @brief Per-voxel log-odds occupancy, length voxel_count_.
     */
    std::vector<double> occupancy_buffer_;

    /**
     * @brief Per-voxel distance in metres (kUndefined until first fusion).
     */
    std::vector<double> distance_buffer_;

    /**
     * @brief Hits since last UpdateOccupancy; reset to 0 after fusion.
     */
    std::vector<int> hit_count_;

    /**
     * @brief Total observations (hits + misses) since last UpdateOccupancy.
     */
    std::vector<int> observation_count_;

    /**
     * @brief Closest occupied voxel coordinates for each cell, or kUndefined.
     */
    std::vector<Eigen::Vector3i> closest_obstacle_;

    /**
     * @brief Head of the doubly linked list of cells sharing that closest obstacle.
     *
     * Length voxel_count_ + 1; the last slot is undefined_voxel_index_.
     */
    std::vector<int> obstacle_list_head_;

    /**
     * @brief Previous index in the closest-obstacle list, or kUndefined.
     */
    std::vector<int> previous_list_index_;

    /**
     * @brief Next index in the closest-obstacle list, or kUndefined.
     */
    std::vector<int> next_list_index_;

    /**
     * @brief Voxels that became occupied; consumed by UpdateESDF.
     */
    std::queue<QueueElement> insert_queue_;

    /**
     * @brief Voxels that became free; consumed by UpdateESDF.
     */
    std::queue<QueueElement> delete_queue_;

    /**
     * @brief BFS frontier of distance updates.
     */
    std::queue<QueueElement> update_queue_;

    /**
     * @brief Voxels with pending hit/miss counts; consumed by UpdateOccupancy.
     */
    std::queue<QueueElement> occupancy_queue_;
};

}  // namespace esdf
}  // namespace map
}  // namespace autonomy

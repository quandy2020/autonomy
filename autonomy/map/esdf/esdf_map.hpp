/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *
 * ROS-free port of FIESTA incremental ESDF (IROS 2019, arXiv:1903.02144).
 * Dense array + probabilistic occupancy + insert/delete BFS.
 */

#pragma once

#include <queue>
#include <vector>

#include <Eigen/Core>

#include "autonomy/map/esdf/options.hpp"

namespace autonomy {
namespace map {
namespace esdf {

/** @brief Index / distance sentinels used by the FIESTA buffers. */
inline constexpr int kUndefined = -10000;
inline constexpr int kInfinity = 10000;

/**
 * @brief Incremental Euclidean signed-distance field on a fixed 3D grid.
 *
 * Observe with SetOccupancy or IntegrateRay, then UpdateOccupancy + UpdateESDF.
 * Distance is metres to the nearest occupied voxel centre; occupied cells are 0.
 */
class EsdfMap {
public:
    explicit EsdfMap(Options options = {});

    /** @brief Log-odds occupancy fusion (called from the constructor). */
    void SetProbabilities(double p_hit, double p_miss, double p_min, double p_max,
                          double p_occ);

    /**
     * @brief Count one observation at a world position.
     * @return Voxel linear index, or kUndefined if outside the map.
     */
    int SetOccupancy(const Eigen::Vector3d& pos, bool occupied);
    int SetOccupancy(const Eigen::Vector3i& vox, bool occupied);

    /** @brief 1 occupied, 0 free, kUndefined out of map. */
    int GetOccupancy(const Eigen::Vector3d& pos) const;
    int GetOccupancy(const Eigen::Vector3i& vox) const;

    /** @brief Distance in metres; kInfinity if the voxel was never updated. */
    double GetDistance(const Eigen::Vector3d& pos) const;
    double GetDistance(const Eigen::Vector3i& vox) const;

    /**
     * @brief Trilinear distance and gradient at a world point.
     * @return Distance, or -1 if pos is outside the map.
     */
    double GetDistanceWithGradient(const Eigen::Vector3d& pos,
                                   Eigen::Vector3d* grad) const;

    /** @brief True when occupancy observations are waiting. */
    bool CheckUpdate() const;

    /**
     * @brief Fuse hit/miss counts into log-odds and fill insert/delete queues.
     * @return True when ESDF insert or delete work remains.
     */
    bool UpdateOccupancy(bool global_map);

    /** @brief Incremental BFS ESDF from insert_queue_ and delete_queue_. */
    void UpdateESDF();

    /** @brief Restrict BFS and visualization to an AABB (world metres). */
    void SetUpdateRange(const Eigen::Vector3d& min_pos,
                        const Eigen::Vector3d& max_pos, bool new_vec = true);
    void SetOriginalRange();

    /**
     * @brief One FIESTA ray: endpoint occupied (or clamped free), free along the
     * ray. Points are already in the map/world frame.
     */
    void IntegrateRay(const Eigen::Vector3d& origin, const Eigen::Vector3d& point);

    void IntegrateCloud(const Eigen::Vector3d& origin,
                        const std::vector<Eigen::Vector3d>& points);

    /** @brief Occupied voxel centres in the current update AABB. */
    std::vector<Eigen::Vector3d> OccupiedPoints() const;

    /** @brief DLL consistency of closest-obstacle lists. */
    bool CheckConsistency() const;

    int grid_total_size() const { return grid_total_size_; }
    const Options& options() const { return options_; }

private:
    struct QueueElement {
        Eigen::Vector3i point{0, 0, 0};
        double distance{0.0};
    };

    static double Logit(double x);

    bool Exist(int idx) const;
    bool PosInMap(const Eigen::Vector3d& pos) const;
    bool VoxInRange(const Eigen::Vector3i& vox, bool current_vec = true) const;
    void Pos2Vox(const Eigen::Vector3d& pos, Eigen::Vector3i* vox) const;
    void Vox2Pos(const Eigen::Vector3i& vox, Eigen::Vector3d* pos) const;
    int Vox2Idx(const Eigen::Vector3i& vox) const;
    Eigen::Vector3i Idx2Vox(int idx) const;
    double Dist(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const;
    double StoredDistance(int idx) const;

    void DeleteFromList(int link, int idx);
    void InsertIntoList(int link, int idx);

    Options options_;
    double resolution_inv_{10.0};
    double prob_hit_log_{0.0};
    double prob_miss_log_{0.0};
    double clamp_min_log_{0.0};
    double clamp_max_log_{0.0};
    double min_occupancy_log_{0.0};

    Eigen::Vector3d origin_{0, 0, 0};
    Eigen::Vector3d min_range_{0, 0, 0};
    Eigen::Vector3d max_range_{0, 0, 0};
    Eigen::Vector3i grid_size_{0, 0, 0};
    int grid_size_yz_{0};
    int grid_total_size_{0};
    int reserved_idx_4_undefined_{0};

    Eigen::Vector3i min_vec_{0, 0, 0};
    Eigen::Vector3i max_vec_{0, 0, 0};
    Eigen::Vector3i last_min_vec_{0, 0, 0};
    Eigen::Vector3i last_max_vec_{0, 0, 0};

    std::vector<double> occupancy_buffer_;
    std::vector<double> distance_buffer_;
    std::vector<int> num_hit_;
    std::vector<int> num_miss_;
    std::vector<Eigen::Vector3i> closest_obstacle_;
    std::vector<int> head_;
    std::vector<int> prev_;
    std::vector<int> next_;

    std::queue<QueueElement> insert_queue_;
    std::queue<QueueElement> delete_queue_;
    std::queue<QueueElement> update_queue_;
    std::queue<QueueElement> occupancy_queue_;
};

}  // namespace esdf
}  // namespace map
}  // namespace autonomy

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
 * @file branch_and_bound.hpp
 * @brief Hierarchical pose search on an occupancy prior (SE2 / SE3-yaw / SE3).
 */

#ifndef ARGUS_RELOCALIZATION_BRANCH_AND_BOUND_HPP_
#define ARGUS_RELOCALIZATION_BRANCH_AND_BOUND_HPP_

#include <atomic>
#include <memory>
#include <vector>

#include "argus/common/macros.hpp"
#include "argus/relocalization/prior_map_loader.hpp"
#include "argus/relocalization/relocalization_base.hpp"

namespace argus {
namespace relocalization {

enum class BnBSearchSpace {
    /** @brief Planar: x, y, yaw (z from guess / map mid). */
    kSE2 = 0,
    /** @brief 4-DoF coarse + optional roll/pitch refine. */
    kSE3Yaw = 1,
    /** @brief 4-DoF coarse + full 6-DoF local coordinate descent. */
    kSE3 = 2,
};

/** @brief Parameters for hierarchical branch-and-bound relocalization. */
struct BranchAndBoundConfig {
    int max_depth = 6;       /**< @brief Maximum BnB tree depth. */
    double resolution = 0.5; /**< @brief Coarse xy step (m). */
    double z_resolution =
        0.5; /**< @brief Coarse z step (m); used when ≥ kSE3Yaw. */
    double yaw_step_rad = 0.2;    /**< @brief Coarse yaw step (rad). */
    double score_threshold = 0.7; /**< @brief Minimum score to accept a pose. */
    /** @brief Expand search around initial guess (± this many coarse steps). */
    int search_radius_steps = 8;
    /** @brief Z search radius in coarse steps (≥ kSE3Yaw). */
    int search_radius_z_steps = 4;
    /** @brief Max nodes expanded (soft budget). */
    int max_expansions = 20000;
    BnBSearchSpace search_space =
        BnBSearchSpace::kSE3; /**< @brief Search dimensionality. */
    /**
     * @brief After xyz+yaw, local grid refine of roll/pitch (rad half-range).
     *        Used for kSE3Yaw; for kSE3 seeds the 6-DoF local search.
     */
    double roll_pitch_refine_rad = 0.12;
    /** @brief Samples per axis in [-refine, +refine] (odd recommended). */
    int roll_pitch_steps = 3;
    /** @brief kSE3: translation refine half-range (m). */
    double se3_trans_refine_m = 0.25;
    /** @brief kSE3: yaw refine half-range (rad). */
    double se3_yaw_refine_rad = 0.15;
    /** @brief kSE3: coordinate-descent passes over 6 DoF. */
    int se3_refine_passes = 2;
};

/**
 * @brief Hierarchical search: score query cloud vs PriorOccupancyMap.
 *
 * Coarse levels search xyz+yaw (or xy+yaw); kSE3 finishes with local 6-DoF
 * coordinate descent so production reloc recovers small roll/pitch tilt.
 */
class BranchAndBound
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(BranchAndBound);

    /**
     * @brief Construct searcher with occupancy prior and tuning parameters.
     * @param map Prior occupancy map for scoring.
     * @param config BnB search configuration (default constructed).
     */
    BranchAndBound(PriorOccupancyMap::ConstSharedPtr map,
                   BranchAndBoundConfig config = {});

    /**
     * @brief Run hierarchical pose search for a relocalization request.
     * @param request Query cloud, initial guess, and options.
     * @param cancel Optional flag; set true to abort early.
     * @return Best pose and score, or failure if below threshold.
     */
    RelocResult Search(const RelocRequest& request, std::atomic<bool>* cancel);

    /** @return Current configuration (read-only). */
    const BranchAndBoundConfig& config() const {
        return config_;
    }

private:
    struct Node {
        double x = 0;
        double y = 0;
        double z = 0;
        double yaw = 0;
        double half_xy = 0;
        double half_z = 0;
        double half_yaw = 0;
        int depth = 0;
        double score = 0;
    };

    transform::Rigid3d MakePose(double x, double y, double z, double yaw) const;
    transform::Rigid3d MakePoseFull(double x, double y, double z, double roll,
                                    double pitch, double yaw) const;
    double ScoreAt(const sensor::PointCloudXYZI& cloud_b, double x, double y,
                double z, double yaw) const;
    double ScoreAtPose(const sensor::PointCloudXYZI& cloud_b,
                    const transform::Rigid3d& T) const;
    transform::Rigid3d RefineRollPitch(const sensor::PointCloudXYZI& cloud_b,
                                       const transform::Rigid3d& T_seed,
                                       double* best_score) const;
    /** @brief Local 6-DoF coordinate descent around seed. */
    transform::Rigid3d RefineSE3(const sensor::PointCloudXYZI& cloud_b,
                                 const transform::Rigid3d& T_seed,
                                 double* best_score) const;

    bool SeedSearchNodes(const RelocRequest& request, std::atomic<bool>* cancel,
                         std::vector<Node>* seeds, RelocResult* out);
    void ExpandChildren(const Node& cur, bool search_z,
                        const sensor::PointCloudXYZI& cloud,
                        std::vector<Node>* children) const;

    PriorOccupancyMap::ConstSharedPtr map_;
    BranchAndBoundConfig config_;
};

}  // namespace relocalization
}  // namespace argus

#endif  // ARGUS_RELOCALIZATION_BRANCH_AND_BOUND_HPP_

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
 * @file pose_graph.hpp
 * @brief Async SE(3) pose graph with Snapshot / FPR try-accept (Ceres or GN).
 *
 * Worker-thread only — never call Optimize / TryAcceptLoops from ESIKF.
 * Solver priority: GTSAM LM (`ARGUS_HAS_GTSAM`) → Ceres → dense Gauss-Newton.
 * Enable GTSAM with `-DARGUS_USE_GTSAM=ON` when the package is available.
 */

#ifndef ARGUS_BACKEND_POSE_GRAPH_POSE_GRAPH_HPP_
#define ARGUS_BACKEND_POSE_GRAPH_POSE_GRAPH_HPP_

#include <mutex>
#include <unordered_map>
#include <vector>

#include "argus/common/macros.hpp"
#include "argus/common/math/transform/rigid_transform.hpp"

namespace argus {
namespace backend {

/** @brief Loop-closure constraint between two pose nodes. */
struct LoopFactor {
    int from_id = 0;
    int to_id = 0;
    /** @brief Measured pose of `to_id` in frame `from_id`. */
    transform::Rigid3d T_i_j = transform::Rigid3d::Identity();
    double information = 100.0;
};

/** @brief Pose-graph node with world-frame body pose. */
struct PoseGraphNode {
    int id = 0;
    transform::Rigid3d T_w_b = transform::Rigid3d::Identity();
};

/** @brief Hard FPR thresholds (LTA-OM-style optimize-and-recover). */
struct PoseGraphFprConfig {
    bool enable = true;
    /** @brief Reject if max loop ‖Log(e)‖ exceeds this (rad·m mixed SE3). */
    double max_loop_residual = 2.0;
    /** @brief Reject if RMS cost after solve exceeds this (0 = disable). */
    double max_cost_rms = 0.0;
};

/** @brief Outcome of Optimize / TryAcceptLoops. */
struct PoseGraphOptimizeResult {
    bool ok = false;
    bool rejected_by_fpr = false;
    double cost_rms = 0.0;
    double max_loop_residual = 0.0;
    int iterations = 0;
};

/**
 * @brief Full graph snapshot for FPR rollback (poses + edges).
 * @note Copy is intentional; graphs stay modest (keyframes).
 */
struct PoseGraphSnapshot {
    std::unordered_map<int, transform::Rigid3d> poses;
    struct Edge {
        int from_id = 0;
        int to_id = 0;
        transform::Rigid3d T_i_j = transform::Rigid3d::Identity();
        double information = 1.0;
        bool is_loop = false;
    };
    std::vector<Edge> edges;
    int loop_factor_count = 0;
    double last_cost = 0.0;
};

/**
 * @brief SE(3) pose graph for loop closure (async worker only).
 */
class PoseGraph
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(PoseGraph);

    void Clear();

    void AddNode(int id, const transform::Rigid3d& T_w_b);
    void SetPose(int id, const transform::Rigid3d& T_w_b);
    void AddOdometryEdge(int from_id, int to_id, const transform::Rigid3d& T_i_j,
                         double information = 50.0);
    void AddLoopFactor(const LoopFactor& factor);

    /**
     * @brief Optimize all edges. Gauge-fixes lowest node id.
     * @param max_iterations Solver iteration cap.
     * @return Cost / residual summary (`ok` if solve finished finitely).
     */
    PoseGraphOptimizeResult Optimize(int max_iterations = 10);

    /**
     * @brief Snapshot → add trial loops → Optimize → FPR gate → Restore or keep.
     * @param trial Loop factors under test (not yet committed on reject).
     * @param fpr Hard residual / cost gates.
     * @param max_iterations Solver iteration cap.
     * @return `ok` if accepted; on reject graph is unchanged.
     */
    PoseGraphOptimizeResult TryAcceptLoops(const std::vector<LoopFactor>& trial,
                                           const PoseGraphFprConfig& fpr,
                                           int max_iterations = 10);

    /** @return Deep copy of poses and edges (for FPR / tests). */
    PoseGraphSnapshot Snapshot() const;
    /** @brief Replace graph state from a snapshot. */
    void Restore(const PoseGraphSnapshot& snapshot);

    bool HasNode(int id) const;
    transform::Rigid3d Pose(int id) const;
    std::vector<PoseGraphNode> Nodes() const;
    int loop_factor_count() const;
    double last_cost() const;

    /**
     * @brief Max SE(3) log residual over loop edges (post-optimize diagnostic).
     * @return 0 if no loop edges.
     */
    double MaxLoopResidual() const;

private:
    struct Edge {
        int from_id = 0;
        int to_id = 0;
        transform::Rigid3d T_i_j = transform::Rigid3d::Identity();
        double information = 1.0;
        bool is_loop = false;
    };

    PoseGraphOptimizeResult OptimizeUnlocked(int max_iterations);
    double MaxLoopResidualUnlocked() const;
    double CostRmsUnlocked() const;

    mutable std::mutex mutex_;
    std::unordered_map<int, transform::Rigid3d> poses_;
    std::vector<Edge> edges_;
    int loop_factor_count_ = 0;
    double last_cost_ = 0.0;
};

}  // namespace backend
}  // namespace argus

#endif  // ARGUS_BACKEND_POSE_GRAPH_POSE_GRAPH_HPP_

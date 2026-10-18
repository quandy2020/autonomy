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
 * @brief Async pose-graph nodes / edges (SE(3) Gauss-Newton, no Ceres).
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
    int from_id = 0; /**< @brief Source node id. */
    int to_id = 0;   /**< @brief Target node id. */
    /** @brief Measured pose of `to_id` in frame `from_id`. */
    transform::Rigid3d T_i_j = transform::Rigid3d::Identity();
    /** @brief Scalar information weight on 6-DoF residual (trans + rot). */
    double information = 100.0;
};

/** @brief Pose-graph node with world-frame body pose. */
struct PoseGraphNode {
    int id = 0; /**< @brief Unique node identifier. */
    transform::Rigid3d T_w_b =
        transform::Rigid3d::Identity(); /**< @brief Body pose in world frame. */
};

class PoseGraph
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(PoseGraph);

    /** @brief Remove all nodes and edges. */
    void Clear();

    /**
     * @brief Add or replace a pose node.
     * @param id Node identifier.
     * @param T_w_b Body pose in world frame.
     */
    void AddNode(int id, const transform::Rigid3d& T_w_b);

    /**
     * @brief Overwrite pose if node exists (gauge / anchor refresh).
     * @param id Existing node id.
     * @param T_w_b New body pose in world frame.
     */
    void SetPose(int id, const transform::Rigid3d& T_w_b);

    /**
     * @brief Add odometry relative pose constraint.
     * @param from_id Source node id.
     * @param to_id Target node id.
     * @param T_i_j Measured pose of `to_id` in frame `from_id`.
     * @param information Scalar information weight (default 50).
     */
    void AddOdometryEdge(int from_id, int to_id,
                         const transform::Rigid3d& T_i_j,
                         double information = 50.0);

    /** @param factor Loop-closure constraint to append. */
    void AddLoopFactor(const LoopFactor& factor);

    /**
     * @brief SE(3) Gauss-Newton on worker thread only — never call from ESIKF.
     *
     * Residual: \(e=\mathrm{Log}(T_{ij}^{-1} T_i^{-1} T_j)\in\mathbb{R}^6\).
     * First node (lowest id) is gauge-fixed.
     * @param max_iterations Maximum GN iterations (default 10).
     */
    void Optimize(int max_iterations = 10);

    /**
     * @param id Node id.
     * @return True if a node with @p id exists.
     */
    bool HasNode(int id) const;

    /**
     * @param id Node id.
     * @return Current optimized pose (identity if missing).
     */
    transform::Rigid3d Pose(int id) const;

    /** @return Snapshot of all nodes and poses. */
    std::vector<PoseGraphNode> Nodes() const;

    /** @return Number of loop-closure edges added. */
    int loop_factor_count() const;

    /** @return Last Optimize residual RMS (0 if never run). */
    double last_cost() const;

private:
    struct Edge {
        int from_id = 0;
        int to_id = 0;
        transform::Rigid3d T_i_j = transform::Rigid3d::Identity();
        double information = 1.0;
        bool is_loop = false;
    };

    mutable std::mutex mutex_;
    std::unordered_map<int, transform::Rigid3d> poses_;
    std::vector<Edge> edges_;
    int loop_factor_count_ = 0;
    double last_cost_ = 0.0;
};

}  // namespace backend
}  // namespace argus

#endif  // ARGUS_BACKEND_POSE_GRAPH_POSE_GRAPH_HPP_

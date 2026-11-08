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
 * @file loop_pose_graph.hpp
 * @brief Async SE(3) pose graph with Snapshot / loop-gate try-accept.
 * Worker-thread only — never call Optimize / TryAcceptLoops from the
 * front-end filter thread. Pose-graph optimization is provided by the
 * required GTSAM backend; GTSAM absence is a configuration error.
 */

#ifndef ATLAS_BACKEND_POSE_GRAPH_LOOP_POSE_GRAPH_HPP_
#define ATLAS_BACKEND_POSE_GRAPH_LOOP_POSE_GRAPH_HPP_

#include <array>
#include <cstdint>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "backend/pose_graph/loop_factor.hpp"
#include "common/macros.hpp"
#include "common/transform/rigid_transform.hpp"

namespace atlas {
namespace backend {
namespace pose_graph {

/**
 * @brief Pose-graph node with world-frame body pose.
 * @var id    Node identifier.
 * @var T_w_b Body pose in the world frame.
 */
struct LoopPoseGraphNode {
  int id = 0;
  transform::Rigid3d T_w_b = transform::Rigid3d::Identity();
};

/**
 * @brief Hard residual / cost gates for loop try-accept (optimize-and-recover).
 * @var enable             When true, reject loops that fail the gates.
 * @var max_loop_residual  Reject if max loop ‖Log(e)‖ exceeds this
 *                         (rad·m mixed SE3).
 * @var max_mean_cost      Reject if root-mean cost after solve exceeds this
 *                         (0 = disable the cost gate).
 */
struct LoopGateConfig {
  bool enable = true;
  double max_loop_residual = 2.0;
  double max_mean_cost = 0.0;
};

/**
 * @brief Outcome of @ref LoopPoseGraph::Optimize / @ref
 * LoopPoseGraph::TryAcceptLoops.
 * @var ok                  True when the solve finished and (for try-accept)
 *                          the loop gate accepted the result.
 * @var rejected_by_gate    True when the loop gate rolled back the trial.
 * @var rejected_as_stale   True when an optimized snapshot no longer matches
 *                          the current graph version.
 * @var root_mean_cost      Root-mean residual cost over active edges.
 * @var max_loop_residual   Max SE(3) log residual over loop edges.
 * @var iterations          Solver iterations performed.
 */
struct OptimizeResult {
  bool ok = false;
  bool rejected_by_gate = false;
  bool rejected_as_stale = false;
  double root_mean_cost = 0.0;
  double max_loop_residual = 0.0;
  int iterations = 0;
};

/**
 * @brief Full graph snapshot for loop-gate rollback (poses + edges).
 * Copy is intentional; graphs stay modest (keyframes).
 * @var poses             World-frame body poses keyed by node id.
 * @var edges             Relative edges including loop flags.
 * @var loop_factor_count Number of loop edges at snapshot time.
 * @var last_cost         Last recorded root-mean cost.
 * @var state_version     Monotonic graph version at snapshot time.
 */
struct LoopPoseGraphSnapshot {
  std::unordered_map<int, transform::Rigid3d> poses;
  /**
   * @brief One relative edge in a snapshot.
   * @var from_id     Source node identifier.
   * @var to_id       Target node identifier.
   * @var T_i_j       Measured relative pose.
   * @var information Scalar information weight.
   * @var is_loop     True for loop-closure edges.
   */
  struct Edge {
    int from_id = 0;
    int to_id = 0;
    transform::Rigid3d T_i_j = transform::Rigid3d::Identity();
    double information = 1.0;
    bool is_loop = false;
    std::array<double, 6> diagonal_information{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
    bool has_diagonal_information = false;
  };
  std::vector<Edge> edges;
  int loop_factor_count = 0;
  double last_cost = 0.0;
  std::uint64_t state_version = 0;
};

/**
 * @brief SE(3) pose graph for loop closure (async worker only).
 */
class LoopPoseGraph {
 public:
  ATLAS_SHARED_PTR_DEFINITIONS(LoopPoseGraph);

  /**
   * @brief Remove all nodes and edges; reset cost counters.
   */
  void Clear();

  /**
   * @brief Insert or replace a node pose.
   * @param[in] id    Node identifier.
   * @param[in] T_w_b Body pose in the world frame.
   */
  void AddNode(int id, const transform::Rigid3d& T_w_b);

  /**
   * @brief Update an existing node pose; no-op if @p id is missing.
   * @param[in] id    Node identifier.
   * @param[in] T_w_b New body pose in the world frame.
   */
  void SetPose(int id, const transform::Rigid3d& T_w_b);

  /**
   * @brief Add an odometry (non-loop) relative edge.
   * @param[in] from_id     Source node identifier.
   * @param[in] to_id       Target node identifier.
   * @param[in] T_i_j       Measured pose of @p to_id in @p from_id frame.
   * @param[in] information Scalar information weight (default 50).
   */
  void AddOdometryEdge(int from_id, int to_id, const transform::Rigid3d& T_i_j,
                       double information = 50.0);
  void AddOdometryEdge(int from_id, int to_id, const transform::Rigid3d& T_i_j,
                       const std::array<double, 6>& diagonal_information);

  /**
   * @brief Add a committed loop-closure factor.
   * @param[in] factor Loop edge to append (increments loop count).
   */
  void AddLoopFactor(const LoopFactor& factor);

  /**
   * @brief Optimize all edges; gauge-fixes the lowest node in each component.
   * @param[in] max_iterations Solver iteration cap (default 10).
   * @return Cost / residual summary; @c ok is true if the solve finished
   *         finitely.
   */
  OptimizeResult Optimize(int max_iterations = 10);

  /**
   * @brief Snapshot → add trial loops → solve → gate → version-checked commit.
   * @param[in] trial          Loop factors under test (not committed on
   *                           reject).
   * @param[in] gate           Hard residual / cost gates.
   * @param[in] max_iterations Solver iteration cap (default 10).
   * @return @c ok true if accepted; rejected trials leave the graph unchanged.
   *         Concurrent graph changes set @c rejected_as_stale.
   */
  OptimizeResult TryAcceptLoops(const std::vector<LoopFactor>& trial,
                                const LoopGateConfig& gate,
                                int max_iterations = 10);

  /**
   * @brief Deep copy of poses and edges (for loop-gate / tests).
   * @return Snapshot of the current graph state.
   */
  LoopPoseGraphSnapshot Snapshot() const;

  /**
   * @brief Commit a complete optimized pose set for a captured graph version.
   *
   * The operation is atomic. It fails without changing state if the graph
   * changed since the snapshot, if any node is missing, or if a pose is
   * invalid. A successful commit advances the graph version.
   * @param[in] expected_state_version Version stored in the source snapshot.
   * @param[in] optimized_poses Complete optimized poses keyed by node id.
   * @param[in] root_mean_cost Solver objective summary for this result.
   * @return True when the result was committed to the current graph.
   */
  bool CommitPoses(
      std::uint64_t expected_state_version,
      const std::unordered_map<int, transform::Rigid3d>& optimized_poses,
      double root_mean_cost);

  /**
   * @brief Replace graph state from a snapshot.
   * @param[in] snapshot Previously captured @ref LoopPoseGraphSnapshot.
   */
  void Restore(const LoopPoseGraphSnapshot& snapshot);

  /**
   * @brief Whether a node with @p id exists.
   * @param[in] id Node identifier.
   * @return True when the node is present.
   */
  bool HasNode(int id) const;

  /**
   * @brief World-frame body pose for @p id.
   * @param[in] id Node identifier.
   * @return Stored pose, or identity if @p id is missing.
   */
  transform::Rigid3d Pose(int id) const;

  /**
   * @brief All nodes as @ref LoopPoseGraphNode values.
   * @return Vector of id / pose pairs (unordered).
   */
  std::vector<LoopPoseGraphNode> Nodes() const;

  /**
   * @brief Number of committed loop-closure edges.
   * @return Loop factor count.
   */
  int loop_factor_count() const;

  /**
   * @brief Last recorded root-mean residual cost.
   * @return Cost from the most recent successful optimize.
   */
  double last_cost() const;

  /**
   * @brief Max SE(3) log residual over loop edges (post-optimize diagnostic).
   * @return Maximum residual, or 0 if there are no loop edges.
   */
  double MaxLoopResidual() const;

  /**
   * @brief Version of the current externally visible graph state.
   *
   * The version advances after each successful mutation or optimization.
   * Restoring a snapshot advances the current version instead of rewinding it.
   * @return Monotonic state version for stale-result detection.
   */
  std::uint64_t state_version() const;

 private:
  struct Edge {
    int from_id = 0;
    int to_id = 0;
    transform::Rigid3d T_i_j = transform::Rigid3d::Identity();
    double information = 1.0;
    bool is_loop = false;
    std::array<double, 6> diagonal_information{{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}};
    bool has_diagonal_information = false;
  };

  OptimizeResult OptimizeUnlocked(int max_iterations);
  double MaxLoopResidualUnlocked() const;

  mutable std::mutex mutex_;
  std::unordered_map<int, transform::Rigid3d> poses_;
  std::vector<Edge> edges_;
  int loop_factor_count_ = 0;
  double last_cost_ = 0.0;
  std::uint64_t state_version_ = 0;
};

}  // namespace pose_graph
}  // namespace backend
}  // namespace atlas

#endif  // ATLAS_BACKEND_POSE_GRAPH_LOOP_POSE_GRAPH_HPP_

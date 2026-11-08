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

#include "backend/pose_graph/loop_pose_graph.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "backend/pose_graph/pose_graph_solver.hpp"

namespace atlas {
namespace backend {
namespace pose_graph {

void LoopPoseGraph::Clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  poses_.clear();
  edges_.clear();
  loop_factor_count_ = 0;
  last_cost_ = 0.0;
  ++state_version_;
}

void LoopPoseGraph::AddNode(int id, const transform::Rigid3d& T_w_b) {
  std::lock_guard<std::mutex> lock(mutex_);
  poses_[id] = T_w_b;
  ++state_version_;
}

void LoopPoseGraph::SetPose(int id, const transform::Rigid3d& T_w_b) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (poses_.count(id) == 0) {
    return;
  }
  poses_[id] = T_w_b;
  ++state_version_;
}

void LoopPoseGraph::AddOdometryEdge(int from_id, int to_id,
                                    const transform::Rigid3d& T_i_j,
                                    double information) {
  std::lock_guard<std::mutex> lock(mutex_);
  edges_.push_back(Edge{from_id, to_id, T_i_j, information, false});
  ++state_version_;
}

void LoopPoseGraph::AddOdometryEdge(
    int from_id, int to_id, const transform::Rigid3d& T_i_j,
    const std::array<double, 6>& diagonal_information) {
  std::lock_guard<std::mutex> lock(mutex_);
  Edge edge{from_id, to_id, T_i_j, 1.0, false};
  edge.diagonal_information = diagonal_information;
  edge.has_diagonal_information = true;
  edges_.push_back(std::move(edge));
  ++state_version_;
}

void LoopPoseGraph::AddLoopFactor(const LoopFactor& factor) {
  std::lock_guard<std::mutex> lock(mutex_);
  Edge edge{factor.from_id, factor.to_id, factor.T_i_j, factor.information,
            true};
  edge.diagonal_information = factor.diagonal_information;
  edge.has_diagonal_information = factor.has_diagonal_information;
  edges_.push_back(std::move(edge));
  ++loop_factor_count_;
  ++state_version_;
}

LoopPoseGraphSnapshot LoopPoseGraph::Snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  LoopPoseGraphSnapshot snap;
  snap.poses = poses_;
  snap.edges.reserve(edges_.size());
  for (const Edge& e : edges_) {
    LoopPoseGraphSnapshot::Edge edge{e.from_id, e.to_id, e.T_i_j, e.information,
                                     e.is_loop};
    edge.diagonal_information = e.diagonal_information;
    edge.has_diagonal_information = e.has_diagonal_information;
    snap.edges.push_back(std::move(edge));
  }
  snap.loop_factor_count = loop_factor_count_;
  snap.last_cost = last_cost_;
  snap.state_version = state_version_;
  return snap;
}

bool LoopPoseGraph::CommitPoses(
    std::uint64_t expected_state_version,
    const std::unordered_map<int, transform::Rigid3d>& optimized_poses,
    double root_mean_cost) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (expected_state_version != state_version_ ||
      optimized_poses.size() != poses_.size() ||
      !std::isfinite(root_mean_cost) || root_mean_cost < 0.0) {
    return false;
  }
  for (const auto& entry : poses_) {
    const auto optimized_pose = optimized_poses.find(entry.first);
    const double rotation_norm =
        optimized_pose == optimized_poses.end()
            ? 0.0
            : optimized_pose->second.rotation().squaredNorm();
    if (optimized_pose == optimized_poses.end() ||
        !optimized_pose->second.translation().allFinite() ||
        !optimized_pose->second.rotation().coeffs().allFinite() ||
        !std::isfinite(rotation_norm) || rotation_norm < 1e-24) {
      return false;
    }
  }
  for (auto& entry : poses_) {
    entry.second = optimized_poses.at(entry.first);
  }
  last_cost_ = root_mean_cost;
  ++state_version_;
  return true;
}

void LoopPoseGraph::Restore(const LoopPoseGraphSnapshot& snapshot) {
  std::lock_guard<std::mutex> lock(mutex_);
  poses_ = snapshot.poses;
  edges_.clear();
  edges_.reserve(snapshot.edges.size());
  for (const auto& e : snapshot.edges) {
    Edge edge{e.from_id, e.to_id, e.T_i_j, e.information, e.is_loop};
    edge.diagonal_information = e.diagonal_information;
    edge.has_diagonal_information = e.has_diagonal_information;
    edges_.push_back(std::move(edge));
  }
  loop_factor_count_ = snapshot.loop_factor_count;
  last_cost_ = snapshot.last_cost;
  ++state_version_;
}

bool LoopPoseGraph::HasNode(int id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return poses_.count(id) > 0;
}

transform::Rigid3d LoopPoseGraph::Pose(int id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = poses_.find(id);
  if (it == poses_.end()) {
    return transform::Rigid3d::Identity();
  }
  return it->second;
}

std::vector<LoopPoseGraphNode> LoopPoseGraph::Nodes() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<LoopPoseGraphNode> nodes;
  nodes.reserve(poses_.size());
  for (const auto& kv : poses_) {
    nodes.push_back(LoopPoseGraphNode{kv.first, kv.second});
  }
  return nodes;
}

int LoopPoseGraph::loop_factor_count() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return loop_factor_count_;
}

double LoopPoseGraph::last_cost() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_cost_;
}

double LoopPoseGraph::MaxLoopResidualUnlocked() const {
  double max_r = 0.0;
  for (const Edge& e : edges_) {
    if (!e.is_loop) {
      continue;
    }
    auto it_i = poses_.find(e.from_id);
    auto it_j = poses_.find(e.to_id);
    if (it_i == poses_.end() || it_j == poses_.end()) {
      continue;
    }
    LoopFactor factor{e.from_id,
                      e.to_id,
                      e.T_i_j,
                      e.information,
                      e.diagonal_information,
                      e.has_diagonal_information};
    max_r =
        std::max(max_r, LoopFactorResidual(it_i->second, it_j->second, factor));
  }
  return max_r;
}

double LoopPoseGraph::MaxLoopResidual() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return MaxLoopResidualUnlocked();
}

std::uint64_t LoopPoseGraph::state_version() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_version_;
}

OptimizeResult LoopPoseGraph::OptimizeUnlocked(int max_iterations) {
  OptimizeResult result;
  std::vector<const Edge*> active_edges;
  active_edges.reserve(edges_.size());
  std::unordered_map<int, std::vector<int>> adjacency;
  for (const Edge& e : edges_) {
    bool valid_information =
        std::isfinite(e.information) &&
        (e.has_diagonal_information || e.information > 0.0);
    if (e.has_diagonal_information) {
      for (double information : e.diagonal_information) {
        valid_information = valid_information && std::isfinite(information) &&
                            information >= 0.0;
      }
    }
    if (e.from_id == e.to_id || poses_.count(e.from_id) == 0 ||
        poses_.count(e.to_id) == 0 || !valid_information ||
        !e.T_i_j.translation().allFinite() ||
        !e.T_i_j.rotation().coeffs().allFinite() ||
        e.T_i_j.rotation().norm() < 1e-12) {
      continue;
    }
    active_edges.push_back(&e);
    adjacency[e.from_id].push_back(e.to_id);
    adjacency[e.to_id].push_back(e.from_id);
  }
  if (active_edges.empty()) {
    return result;
  }

  std::vector<int> node_identifiers;
  node_identifiers.reserve(poses_.size());
  for (const auto& entry : poses_) {
    node_identifiers.push_back(entry.first);
  }
  std::sort(node_identifiers.begin(), node_identifiers.end());
  std::unordered_set<int> fixed_node_identifiers;
  std::unordered_set<int> visited;
  for (int root_identifier : node_identifiers) {
    if (!visited.insert(root_identifier).second) {
      continue;
    }
    fixed_node_identifiers.insert(root_identifier);
    std::queue<int> pending;
    pending.push(root_identifier);
    while (!pending.empty()) {
      const int current_id = pending.front();
      pending.pop();
      for (int neighbor_identifier : adjacency[current_id]) {
        if (visited.insert(neighbor_identifier).second) {
          pending.push(neighbor_identifier);
        }
      }
    }
  }

  FactorGraph problem;
  for (const auto& entry : poses_) {
    if (!problem.AddPose(entry.first, entry.second,
                         fixed_node_identifiers.count(entry.first) != 0)) {
      return result;
    }
  }
  for (const Edge* edge : active_edges) {
    BetweenFactor factor;
    factor.from_id = edge->from_id;
    factor.to_id = edge->to_id;
    factor.relative_pose = edge->T_i_j;
    factor.rotation_weight = edge->information;
    factor.translation_weight = edge->information;
    factor.diagonal_information = edge->diagonal_information;
    factor.diagonal_information_provided = edge->has_diagonal_information;
    problem.AddBetweenFactor(factor);
  }

  SolverOptions options;
  options.maximum_update_count = std::max(1, max_iterations + 1);
  options.convergence_tolerance = 0.01;
  PoseGraphSolver solver;
  const UpdateResult update_result = solver.Update(problem, options);
  if (!update_result.Succeeded()) {
    return result;
  }
  const SolverResult solve_result = solver.CalculateEstimate();
  if (solve_result.Succeeded()) {
    for (const auto& entry : poses_) {
      const auto optimized_pose =
          solve_result.optimized_poses.find(entry.first);
      const double rotation_norm =
          optimized_pose == solve_result.optimized_poses.end()
              ? 0.0
              : optimized_pose->second.rotation().squaredNorm();
      if (optimized_pose == solve_result.optimized_poses.end() ||
          !optimized_pose->second.translation().allFinite() ||
          !optimized_pose->second.rotation().coeffs().allFinite() ||
          !std::isfinite(rotation_norm) || rotation_norm < 1e-24) {
        result.ok = false;
        return result;
      }
    }
    for (auto& entry : poses_) {
      entry.second = solve_result.optimized_poses.at(entry.first);
    }
    result.ok = true;
    result.root_mean_cost = std::sqrt(std::max(0.0, solve_result.final_cost) /
                                      static_cast<double>(active_edges.size()));
    result.iterations = update_result.update_count;
    last_cost_ = result.root_mean_cost;
  }
  result.max_loop_residual = MaxLoopResidualUnlocked();
  return result;
}

OptimizeResult LoopPoseGraph::Optimize(int max_iterations) {
  const LoopPoseGraphSnapshot source_snapshot = Snapshot();
  LoopPoseGraph working_graph;
  working_graph.Restore(source_snapshot);
  OptimizeResult result = working_graph.OptimizeUnlocked(max_iterations);
  if (!result.ok) {
    return result;
  }

  if (!CommitPoses(source_snapshot.state_version, working_graph.poses_,
                   result.root_mean_cost)) {
    result.ok = false;
    result.rejected_as_stale = state_version() != source_snapshot.state_version;
    return result;
  }

  result.max_loop_residual = MaxLoopResidual();
  return result;
}

OptimizeResult LoopPoseGraph::TryAcceptLoops(
    const std::vector<LoopFactor>& trial, const LoopGateConfig& gate,
    int max_iterations) {
  OptimizeResult result;
  if (trial.empty() ||
      (gate.enable &&
       (!std::isfinite(gate.max_loop_residual) ||
        gate.max_loop_residual < 0.0 || !std::isfinite(gate.max_mean_cost) ||
        gate.max_mean_cost < 0.0))) {
    return result;
  }

  const LoopPoseGraphSnapshot source_snapshot = Snapshot();
  LoopPoseGraphSnapshot candidate_snapshot = source_snapshot;
  candidate_snapshot.edges.reserve(source_snapshot.edges.size() + trial.size());
  for (const LoopFactor& factor : trial) {
    const double rotation_norm = factor.T_i_j.rotation().squaredNorm();
    bool valid_factor = factor.from_id != factor.to_id &&
                        source_snapshot.poses.count(factor.from_id) > 0 &&
                        source_snapshot.poses.count(factor.to_id) > 0 &&
                        std::isfinite(factor.information) &&
                        factor.T_i_j.translation().allFinite() &&
                        factor.T_i_j.rotation().coeffs().allFinite() &&
                        std::isfinite(rotation_norm) && rotation_norm >= 1e-24;
    if (!factor.has_diagonal_information) {
      valid_factor = valid_factor && factor.information > 0.0;
    } else {
      double total_information = 0.0;
      for (double information : factor.diagonal_information) {
        valid_factor =
            valid_factor && std::isfinite(information) && information >= 0.0;
        total_information += information;
      }
      valid_factor = valid_factor && total_information > 0.0;
    }
    if (!valid_factor) {
      result.rejected_by_gate = true;
      return result;
    }

    LoopPoseGraphSnapshot::Edge edge;
    edge.from_id = factor.from_id;
    edge.to_id = factor.to_id;
    edge.T_i_j = factor.T_i_j;
    edge.information = factor.information;
    edge.is_loop = true;
    edge.diagonal_information = factor.diagonal_information;
    edge.has_diagonal_information = factor.has_diagonal_information;
    candidate_snapshot.edges.push_back(std::move(edge));
    ++candidate_snapshot.loop_factor_count;
  }

  LoopPoseGraph working_graph;
  working_graph.Restore(candidate_snapshot);
  result = working_graph.OptimizeUnlocked(max_iterations);
  if (!result.ok) {
    result.rejected_by_gate = true;
    return result;
  }

  if (gate.enable) {
    const bool residual_bad = result.max_loop_residual > gate.max_loop_residual;
    const bool cost_bad =
        gate.max_mean_cost > 0.0 && result.root_mean_cost > gate.max_mean_cost;
    if (residual_bad || cost_bad) {
      result.ok = false;
      result.rejected_by_gate = true;
      return result;
    }
  }

  std::unordered_map<int, transform::Rigid3d> optimized_poses =
      working_graph.poses_;
  std::vector<Edge> committed_edges;
  committed_edges.reserve(candidate_snapshot.edges.size());
  for (const LoopPoseGraphSnapshot::Edge& snapshot_edge :
       candidate_snapshot.edges) {
    Edge edge{snapshot_edge.from_id, snapshot_edge.to_id, snapshot_edge.T_i_j,
              snapshot_edge.information, snapshot_edge.is_loop};
    edge.diagonal_information = snapshot_edge.diagonal_information;
    edge.has_diagonal_information = snapshot_edge.has_diagonal_information;
    committed_edges.push_back(std::move(edge));
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (state_version_ != source_snapshot.state_version) {
    result.ok = false;
    result.rejected_as_stale = true;
    return result;
  }
  poses_.swap(optimized_poses);
  edges_.swap(committed_edges);
  loop_factor_count_ = candidate_snapshot.loop_factor_count;
  last_cost_ = result.root_mean_cost;
  ++state_version_;
  result.ok = true;
  result.rejected_by_gate = false;
  return result;
}

}  // namespace pose_graph
}  // namespace backend
}  // namespace atlas

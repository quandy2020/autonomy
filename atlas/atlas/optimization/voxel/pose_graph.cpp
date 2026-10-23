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

#include "optimization/voxel/pose_graph.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "common/math/so3.hpp"
#include "common/transform/transform.hpp"
#include "optimization/voxel/gtsam_optimizer.hpp"
#include "optimization/voxel/pose_graph_internal.hpp"

namespace atlas {
namespace vxslam {
namespace backend {
namespace {

Eigen::Matrix<double, 6, 1> Se3Log(const transform::Rigid3d& T) {
  Eigen::Matrix<double, 6, 1> xi;
  xi.head<3>() = T.translation();
  xi.tail<3>() = math::Log(T.rotation().toRotationMatrix());
  return xi;
}

transform::Rigid3d Se3Exp(const Eigen::Matrix<double, 6, 1>& xi) {
  const Eigen::Vector3d phi(xi(3), xi(4), xi(5));
  return transform::Rigid3d(xi.head<3>(), Eigen::Quaterniond(math::Exp(phi)));
}

transform::Rigid3d RightPlus(const transform::Rigid3d& T,
                             const Eigen::Matrix<double, 6, 1>& delta) {
  return T * Se3Exp(delta);
}

Eigen::Matrix<double, 6, 1> RelativeError(const transform::Rigid3d& Ti,
                                          const transform::Rigid3d& Tj,
                                          const transform::Rigid3d& Tij) {
  return Se3Log(Tij.inverse() * (Ti.inverse() * Tj));
}

void FiniteDiffJacobians(const transform::Rigid3d& Ti,
                         const transform::Rigid3d& Tj,
                         const transform::Rigid3d& Tij, double eps,
                         Eigen::Matrix<double, 6, 6>* Ji,
                         Eigen::Matrix<double, 6, 6>* Jj) {
  *Ji = Eigen::Matrix<double, 6, 6>::Zero();
  *Jj = Eigen::Matrix<double, 6, 6>::Zero();
  for (int d = 0; d < 6; ++d) {
    Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
    delta(d) = eps;
    Ji->col(d) = (RelativeError(RightPlus(Ti, delta), Tj, Tij) -
                  RelativeError(RightPlus(Ti, -delta), Tj, Tij)) /
                 (2.0 * eps);
    Jj->col(d) = (RelativeError(Ti, RightPlus(Tj, delta), Tij) -
                  RelativeError(Ti, RightPlus(Tj, -delta), Tij)) /
                 (2.0 * eps);
  }
}

OptimizeResult OptimizeGaussNewton(
    std::unordered_map<int, transform::Rigid3d>* poses,
    const std::vector<PoseGraphSnapshot::Edge>& edges, int max_iterations) {
  OptimizeResult result;
  if (poses == nullptr || poses->size() < 2 || edges.empty()) {
    return result;
  }

  std::vector<int> ids;
  ids.reserve(poses->size());
  for (const auto& kv : *poses) {
    ids.push_back(kv.first);
  }
  std::sort(ids.begin(), ids.end());
  const int node_count = static_cast<int>(ids.size());
  std::unordered_map<int, int> id_to_index;
  for (int k = 0; k < node_count; ++k) {
    id_to_index[ids[static_cast<std::size_t>(k)]] = k;
  }

  std::vector<transform::Rigid3d> work(static_cast<std::size_t>(node_count));
  for (int k = 0; k < node_count; ++k) {
    work[static_cast<std::size_t>(k)] =
        poses->at(ids[static_cast<std::size_t>(k)]);
    const auto& pose = work[static_cast<std::size_t>(k)];
    if (!pose.translation().allFinite() ||
        !pose.rotation().coeffs().allFinite() ||
        pose.rotation().norm() < 1e-12) {
      return result;
    }
  }

  std::unordered_map<int, std::vector<int>> adjacency;
  int usable_edges = 0;
  for (const auto& edge : edges) {
    if (edge.from_id == edge.to_id || id_to_index.count(edge.from_id) == 0 ||
        id_to_index.count(edge.to_id) == 0 ||
        !std::isfinite(edge.information) || edge.information <= 0.0 ||
        !edge.T_i_j.translation().allFinite() ||
        !edge.T_i_j.rotation().coeffs().allFinite() ||
        edge.T_i_j.rotation().norm() < 1e-12) {
      continue;
    }
    adjacency[edge.from_id].push_back(edge.to_id);
    adjacency[edge.to_id].push_back(edge.from_id);
    ++usable_edges;
  }
  if (usable_edges == 0) {
    return result;
  }

  std::vector<int> anchors;
  std::unordered_set<int> visited;
  for (int root : ids) {
    if (!visited.insert(root).second) {
      continue;
    }
    anchors.push_back(id_to_index.at(root));
    std::queue<int> pending;
    pending.push(root);
    while (!pending.empty()) {
      const int current = pending.front();
      pending.pop();
      for (int neighbor : adjacency[current]) {
        if (visited.insert(neighbor).second) {
          pending.push(neighbor);
        }
      }
    }
  }

  constexpr double kEps = 1e-6;
  int iters = 0;
  for (; iters < max_iterations; ++iters) {
    const int dim = 6 * node_count;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dim, dim);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(dim);
    for (const auto& edge : edges) {
      if (!id_to_index.count(edge.from_id) || !id_to_index.count(edge.to_id) ||
          edge.from_id == edge.to_id || !std::isfinite(edge.information) ||
          edge.information <= 0.0 || !edge.T_i_j.translation().allFinite() ||
          !edge.T_i_j.rotation().coeffs().allFinite() ||
          edge.T_i_j.rotation().norm() < 1e-12) {
        continue;
      }
      const int fi = id_to_index.at(edge.from_id);
      const int ti = id_to_index.at(edge.to_id);
      const auto& Ti = work[static_cast<std::size_t>(fi)];
      const auto& Tj = work[static_cast<std::size_t>(ti)];
      const Eigen::Matrix<double, 6, 1> residual =
          RelativeError(Ti, Tj, edge.T_i_j);
      Eigen::Matrix<double, 6, 6> Ji;
      Eigen::Matrix<double, 6, 6> Jj;
      FiniteDiffJacobians(Ti, Tj, edge.T_i_j, kEps, &Ji, &Jj);
      const double w = edge.information;
      H.block<6, 6>(6 * fi, 6 * fi) += w * Ji.transpose() * Ji;
      H.block<6, 6>(6 * ti, 6 * ti) += w * Jj.transpose() * Jj;
      H.block<6, 6>(6 * fi, 6 * ti) += w * Ji.transpose() * Jj;
      H.block<6, 6>(6 * ti, 6 * fi) += w * Jj.transpose() * Ji;
      b.segment<6>(6 * fi) += w * Ji.transpose() * residual;
      b.segment<6>(6 * ti) += w * Jj.transpose() * residual;
    }
    for (int anchor : anchors) {
      for (int k = 0; k < 6; ++k) {
        const int index = 6 * anchor + k;
        H.row(index).setZero();
        H.col(index).setZero();
        H(index, index) = 1.0;
        b(index) = 0.0;
      }
    }
    const Eigen::VectorXd dx = H.ldlt().solve(-b);
    if (!dx.allFinite()) {
      return result;
    }
    for (int k = 0; k < node_count; ++k) {
      work[static_cast<std::size_t>(k)] =
          RightPlus(work[static_cast<std::size_t>(k)], dx.segment<6>(6 * k));
      if (!work[static_cast<std::size_t>(k)].translation().allFinite() ||
          !work[static_cast<std::size_t>(k)].rotation().coeffs().allFinite()) {
        return result;
      }
    }
    if (dx.norm() < 1e-6) {
      ++iters;
      break;
    }
  }

  int active = 0;
  double final_cost = 0.0;
  for (const auto& edge : edges) {
    if (edge.from_id != edge.to_id && id_to_index.count(edge.from_id) &&
        id_to_index.count(edge.to_id) && std::isfinite(edge.information) &&
        edge.information > 0.0 && edge.T_i_j.translation().allFinite() &&
        edge.T_i_j.rotation().coeffs().allFinite() &&
        edge.T_i_j.rotation().norm() >= 1e-12) {
      const auto residual = RelativeError(
          work[static_cast<std::size_t>(id_to_index.at(edge.from_id))],
          work[static_cast<std::size_t>(id_to_index.at(edge.to_id))],
          edge.T_i_j);
      if (!residual.allFinite()) {
        return result;
      }
      final_cost += edge.information * residual.squaredNorm();
      ++active;
    }
  }
  if (active == 0 || !std::isfinite(final_cost)) {
    return result;
  }
  result.root_mean_cost = std::sqrt(final_cost / static_cast<double>(active));
  result.iterations = iters;
  result.ok = true;
  for (int k = 0; k < node_count; ++k) {
    (*poses)[ids[static_cast<std::size_t>(k)]] =
        work[static_cast<std::size_t>(k)];
  }
  return result;
}

}  // namespace

void PoseGraph::Clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  poses_.clear();
  edges_.clear();
  loop_factor_count_ = 0;
  last_cost_ = 0.0;
}

void PoseGraph::AddNode(int id, const transform::Rigid3d& T_w_b) {
  std::lock_guard<std::mutex> lock(mutex_);
  poses_[id] = T_w_b;
}

void PoseGraph::SetPose(int id, const transform::Rigid3d& T_w_b) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (poses_.count(id) == 0) {
    return;
  }
  poses_[id] = T_w_b;
}

void PoseGraph::AddOdometryEdge(int from_id, int to_id,
                                const transform::Rigid3d& T_i_j,
                                double information) {
  std::lock_guard<std::mutex> lock(mutex_);
  edges_.push_back(Edge{from_id, to_id, T_i_j, information, false});
}

void PoseGraph::AddLoopFactor(const LoopFactor& factor) {
  std::lock_guard<std::mutex> lock(mutex_);
  edges_.push_back(Edge{factor.from_id, factor.to_id, factor.T_i_j,
                        factor.information, true});
  ++loop_factor_count_;
}

PoseGraphSnapshot PoseGraph::Snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  PoseGraphSnapshot snap;
  snap.poses = poses_;
  snap.edges.reserve(edges_.size());
  for (const Edge& e : edges_) {
    snap.edges.push_back(PoseGraphSnapshot::Edge{e.from_id, e.to_id, e.T_i_j,
                                                 e.information, e.is_loop});
  }
  snap.loop_factor_count = loop_factor_count_;
  snap.last_cost = last_cost_;
  return snap;
}

void PoseGraph::Restore(const PoseGraphSnapshot& snapshot) {
  std::lock_guard<std::mutex> lock(mutex_);
  poses_ = snapshot.poses;
  edges_.clear();
  edges_.reserve(snapshot.edges.size());
  for (const auto& e : snapshot.edges) {
    edges_.push_back(
        Edge{e.from_id, e.to_id, e.T_i_j, e.information, e.is_loop});
  }
  loop_factor_count_ = snapshot.loop_factor_count;
  last_cost_ = snapshot.last_cost;
}

bool PoseGraph::HasNode(int id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return poses_.count(id) > 0;
}

transform::Rigid3d PoseGraph::Pose(int id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = poses_.find(id);
  if (it == poses_.end()) {
    return transform::Rigid3d::Identity();
  }
  return it->second;
}

std::vector<PoseGraphNode> PoseGraph::Nodes() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<PoseGraphNode> nodes;
  nodes.reserve(poses_.size());
  for (const auto& kv : poses_) {
    nodes.push_back(PoseGraphNode{kv.first, kv.second});
  }
  return nodes;
}

int PoseGraph::loop_factor_count() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return loop_factor_count_;
}

double PoseGraph::last_cost() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_cost_;
}

double PoseGraph::MaxLoopResidualUnlocked() const {
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
    LoopFactor factor{e.from_id, e.to_id, e.T_i_j, e.information};
    max_r =
        std::max(max_r, LoopFactorResidual(it_i->second, it_j->second, factor));
  }
  return max_r;
}

double PoseGraph::MaxLoopResidual() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return MaxLoopResidualUnlocked();
}

OptimizeResult PoseGraph::OptimizeUnlocked(int max_iterations) {
  OptimizeResult result;

  std::vector<pose_graph_detail::RelativeEdge> solver_edges;
  solver_edges.reserve(edges_.size());
  for (const Edge& e : edges_) {
    solver_edges.push_back(pose_graph_detail::RelativeEdge{
        e.from_id, e.to_id, e.T_i_j, e.information,
        Eigen::Matrix<double, 6, 1>::Ones(), false});
  }

#if defined(ATLAS_HAS_GTSAM) && ATLAS_HAS_GTSAM
  {
    std::unordered_map<int, transform::Rigid3d> out;
    double root_mean_cost = 0.0;
    int iters = 0;
    result.ok = pose_graph_detail::OptimizeWithGtsam(
        poses_, solver_edges, max_iterations, &out, &root_mean_cost, &iters);
    if (result.ok) {
      poses_.swap(out);
      result.root_mean_cost = root_mean_cost;
      result.iterations = iters;
      last_cost_ = root_mean_cost;
    }
  }
#else
  std::vector<PoseGraphSnapshot::Edge> snap_edges;
  snap_edges.reserve(edges_.size());
  for (const Edge& e : edges_) {
    snap_edges.push_back(PoseGraphSnapshot::Edge{e.from_id, e.to_id, e.T_i_j,
                                                 e.information, e.is_loop});
  }
  result = OptimizeGaussNewton(&poses_, snap_edges, max_iterations);
  last_cost_ = result.root_mean_cost;
#endif
  result.max_loop_residual = MaxLoopResidualUnlocked();
  return result;
}

OptimizeResult PoseGraph::Optimize(int max_iterations) {
  std::lock_guard<std::mutex> lock(mutex_);
  return OptimizeUnlocked(max_iterations);
}

OptimizeResult PoseGraph::TryAcceptLoops(const std::vector<LoopFactor>& trial,
                                         const LoopGateConfig& gate,
                                         int max_iterations) {
  std::lock_guard<std::mutex> lock(mutex_);
  OptimizeResult result;
  if (trial.empty()) {
    result.ok = false;
    return result;
  }

  const PoseGraphSnapshot backup = [&]() {
    PoseGraphSnapshot snap;
    snap.poses = poses_;
    snap.edges.reserve(edges_.size());
    for (const Edge& e : edges_) {
      snap.edges.push_back(PoseGraphSnapshot::Edge{e.from_id, e.to_id, e.T_i_j,
                                                   e.information, e.is_loop});
    }
    snap.loop_factor_count = loop_factor_count_;
    snap.last_cost = last_cost_;
    return snap;
  }();

  for (const LoopFactor& f : trial) {
    edges_.push_back(Edge{f.from_id, f.to_id, f.T_i_j, f.information, true});
    ++loop_factor_count_;
  }

  result = OptimizeUnlocked(max_iterations);
  if (!result.ok) {
    poses_ = backup.poses;
    edges_.clear();
    for (const auto& e : backup.edges) {
      edges_.push_back(
          Edge{e.from_id, e.to_id, e.T_i_j, e.information, e.is_loop});
    }
    loop_factor_count_ = backup.loop_factor_count;
    last_cost_ = backup.last_cost;
    result.rejected_by_gate = true;
    return result;
  }

  if (gate.enable) {
    const bool residual_bad = result.max_loop_residual > gate.max_loop_residual;
    const bool cost_bad =
        gate.max_mean_cost > 0.0 && result.root_mean_cost > gate.max_mean_cost;
    if (residual_bad || cost_bad) {
      poses_ = backup.poses;
      edges_.clear();
      for (const auto& e : backup.edges) {
        edges_.push_back(
            Edge{e.from_id, e.to_id, e.T_i_j, e.information, e.is_loop});
      }
      loop_factor_count_ = backup.loop_factor_count;
      last_cost_ = backup.last_cost;
      result.ok = false;
      result.rejected_by_gate = true;
      return result;
    }
  }

  result.ok = true;
  result.rejected_by_gate = false;
  return result;
}

}  // namespace backend
}  // namespace vxslam
}  // namespace atlas

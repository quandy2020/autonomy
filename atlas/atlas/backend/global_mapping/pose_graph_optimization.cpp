/*
 * Copyright 2026 The Openbot Authors
 */
#include "backend/global_mapping/pose_graph_optimization.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include "backend/pose_graph/pose_graph_solver.hpp"
#include "glog/logging.h"

namespace atlas {
namespace backend {
namespace global_mapping {

using ::atlas::frontend::odometry::ScanPosePtr;
using ::atlas::map::Keyframe;

using ::atlas::frontend::inertial::InertialState;
namespace {

std::vector<int> ConnectedSessions(
    int root, const std::vector<const PgoEdges*>& edge_batches) {
  std::unordered_map<int, std::vector<int>> adjacency;
  for (const PgoEdges* batch : edge_batches) {
    if (batch == nullptr) {
      continue;
    }
    for (const PgoEdge& edge : batch->edges) {
      if (edge.m1 < 0 || edge.m2 < 0) {
        continue;
      }
      adjacency[edge.m1].push_back(edge.m2);
      adjacency[edge.m2].push_back(edge.m1);
    }
  }

  std::vector<int> connected{root};
  for (size_t cursor = 0; cursor < connected.size(); ++cursor) {
    const auto found = adjacency.find(connected[cursor]);
    if (found == adjacency.end()) {
      continue;
    }
    for (int neighbor : found->second) {
      if (std::find(connected.begin(), connected.end(), neighbor) ==
          connected.end()) {
        connected.push_back(neighbor);
      }
    }
  }
  std::sort(connected.begin(), connected.end());
  return connected;
}

bool AddLoopEdges(
    const PgoEdges& batch, const std::vector<int>& session_ids,
    const std::vector<int>& stepsizes,
    const std::unordered_map<std::int64_t, transform::Rigid3d>& poses,
    std::vector<pose_graph::BetweenFactor>* factors) {
  if (factors == nullptr || stepsizes.size() != session_ids.size() + 1) {
    return false;
  }
  for (const PgoEdge& edge : batch.edges) {
    std::vector<int> session_indices(2, -1);
    if (!edge.IsAdapt(session_ids, &session_indices)) {
      continue;
    }
    if (edge.ids1.size() != edge.rots.size() ||
        edge.ids2.size() != edge.rots.size() ||
        edge.tras.size() != edge.rots.size() ||
        edge.covs.size() != edge.rots.size()) {
      return false;
    }
    for (size_t i = 0; i < edge.rots.size(); ++i) {
      int from = 0;
      int to = 0;
      if (!ResolvePoseGraphNodeIndex(session_indices[0], edge.ids1[i],
                                     stepsizes, &from) ||
          !ResolvePoseGraphNodeIndex(session_indices[1], edge.ids2[i],
                                     stepsizes, &to) ||
          from == to || poses.count(from) == 0 || poses.count(to) == 0 ||
          !edge.rots[i].allFinite() || !edge.tras[i].allFinite() ||
          !edge.covs[i].allFinite() || edge.covs[i].minCoeff() < 0.0) {
        return false;
      }
      pose_graph::BetweenFactor factor;
      factor.from_id = from;
      factor.to_id = to;
      factor.relative_pose =
          transform::Rigid3d(edge.tras[i], Eigen::Quaterniond(edge.rots[i]));
      for (int axis = 0; axis < 6; ++axis) {
        factor.diagonal_information[static_cast<std::size_t>(axis)] =
            1.0 / std::max(1e-9, edge.covs[i][axis]);
      }
      factor.diagonal_information_provided = true;
      factors->push_back(factor);
    }
  }
  return true;
}

void LogFactorScale(const std::vector<pose_graph::BetweenFactor>& factors,
                    std::size_t begin, std::size_t end, const char* group) {
  double minimum_information = std::numeric_limits<double>::infinity();
  double maximum_information = 0.0;
  std::size_t very_strong_axes = 0;
  std::int64_t strongest_from = -1;
  std::int64_t strongest_to = -1;
  for (std::size_t index = begin; index < end; ++index) {
    const auto& factor = factors[index];
    for (double information : factor.diagonal_information) {
      minimum_information = std::min(minimum_information, information);
      if (information > maximum_information) {
        maximum_information = information;
        strongest_from = factor.from_id;
        strongest_to = factor.to_id;
      }
      very_strong_axes += information > 1e7;
    }
  }
  LOG(WARNING) << group << " factors=" << (end - begin)
               << " information range=[" << minimum_information << ", "
               << maximum_information << "] axes_above_1e7="
               << very_strong_axes << " strongest_edge=" << strongest_from
               << "->" << strongest_to;
}

}  // namespace

bool OptimizePoseGraph(std::vector<std::vector<ScanPosePtr>*>* multimap_scans,
                       std::vector<std::vector<Keyframe*>*>* multimap_keyframes,
                       const PgoEdges& lp_edges, const PgoEdges& gba_edges1,
                       const PgoEdges& gba_edges2, int cur_session_id,
                       bool inject_gba) {
  if (multimap_scans == nullptr || multimap_scans->empty() ||
      cur_session_id < 0 ||
      cur_session_id >= static_cast<int>(multimap_scans->size())) {
    return false;
  }

  std::vector<const PgoEdges*> graph_edges{&lp_edges};
  if (inject_gba) {
    graph_edges.push_back(&gba_edges1);
    graph_edges.push_back(&gba_edges2);
  }
  std::vector<int> session_ids = ConnectedSessions(cur_session_id, graph_edges);
  session_ids.erase(
      std::remove_if(session_ids.begin(), session_ids.end(),
                     [multimap_scans](int id) {
                       return id < 0 ||
                              id >= static_cast<int>(multimap_scans->size()) ||
                              (*multimap_scans)[static_cast<size_t>(id)] ==
                                  nullptr;
                     }),
      session_ids.end());
  if (session_ids.empty() ||
      !std::binary_search(session_ids.begin(), session_ids.end(),
                          cur_session_id)) {
    return false;
  }

  std::vector<int> stepsizes(session_ids.size() + 1, 0);
  for (size_t i = 0; i < session_ids.size(); ++i) {
    const std::size_t session_scan_count =
        (*multimap_scans)[static_cast<size_t>(session_ids[i])]->size();
    if (session_scan_count >
        static_cast<std::size_t>(std::numeric_limits<int>::max() -
                                 stepsizes[i])) {
      return false;
    }
    stepsizes[i + 1] = stepsizes[i] + static_cast<int>(session_scan_count);
  }
  if (stepsizes.back() < 2) {
    return false;
  }

  std::unordered_map<std::int64_t, transform::Rigid3d> initial_poses;
  initial_poses.reserve(static_cast<std::size_t>(stepsizes.back()));
  std::vector<pose_graph::BetweenFactor> factors;
  for (size_t session = 0; session < session_ids.size(); ++session) {
    const auto& scans =
        *(*multimap_scans)[static_cast<size_t>(session_ids[session])];
    const int base = stepsizes[session];
    for (size_t i = 0; i < scans.size(); ++i) {
      const ScanPosePtr& scan = scans[i];
      if (!scan) {
        continue;
      }
      const int node = base + static_cast<int>(i);
      initial_poses.emplace(
          node, transform::Rigid3d(scan->x.p, Eigen::Quaterniond(scan->x.R)));
      if (i == 0 || !scans[i - 1]) {
        continue;
      }
      const InertialState& from = scans[i - 1]->x;
      const InertialState& to = scan->x;
      pose_graph::BetweenFactor factor;
      factor.from_id = node - 1;
      factor.to_id = node;
      factor.relative_pose =
          transform::Rigid3d(from.R.transpose() * (to.p - from.p),
                             Eigen::Quaterniond(from.R.transpose() * to.R));
      const Eigen::Matrix<double, 6, 1>& variances = scans[i - 1]->v6;
      if (!variances.allFinite() || variances.minCoeff() <= 0.0) {
        factor.diagonal_information.fill(1.0 / 1e-4);
      } else {
        for (int axis = 0; axis < 6; ++axis) {
          factor.diagonal_information[static_cast<std::size_t>(axis)] =
              1.0 / std::max(1e-9, variances[axis]);
        }
      }
      factor.diagonal_information_provided = true;
      factors.push_back(factor);
    }
  }

  const std::size_t sequential_factor_count = factors.size();
  if (!AddLoopEdges(lp_edges, session_ids, stepsizes, initial_poses,
                    &factors) ||
      (inject_gba && (!AddLoopEdges(gba_edges1, session_ids, stepsizes,
                                    initial_poses, &factors) ||
                      !AddLoopEdges(gba_edges2, session_ids, stepsizes,
                                    initial_poses, &factors)))) {
    LOG(ERROR) << "OptimizePoseGraph rejected malformed PGO edge data";
    return false;
  }
  if (initial_poses.empty() || factors.empty()) {
    return false;
  }

  std::unordered_map<std::int64_t, std::vector<std::int64_t>> adjacency;
  for (const pose_graph::BetweenFactor& factor : factors) {
    adjacency[factor.from_id].push_back(factor.to_id);
    adjacency[factor.to_id].push_back(factor.from_id);
  }
  std::vector<std::int64_t> node_identifiers;
  node_identifiers.reserve(initial_poses.size());
  for (const auto& entry : initial_poses) {
    node_identifiers.push_back(entry.first);
  }
  std::sort(node_identifiers.begin(), node_identifiers.end());
  std::unordered_set<std::int64_t> fixed_pose_identifiers;
  std::unordered_set<std::int64_t> visited_pose_identifiers;
  for (std::int64_t root_identifier : node_identifiers) {
    if (!visited_pose_identifiers.insert(root_identifier).second) {
      continue;
    }
    fixed_pose_identifiers.insert(root_identifier);
    std::vector<std::int64_t> pending_pose_identifiers{root_identifier};
    for (std::size_t cursor = 0; cursor < pending_pose_identifiers.size();
         ++cursor) {
      for (std::int64_t neighbor_identifier :
           adjacency[pending_pose_identifiers[cursor]]) {
        if (visited_pose_identifiers.insert(neighbor_identifier).second) {
          pending_pose_identifiers.push_back(neighbor_identifier);
        }
      }
    }
  }

  pose_graph::FactorGraph graph;
  for (const auto& entry : initial_poses) {
    if (!graph.AddPose(entry.first, entry.second,
                       fixed_pose_identifiers.count(entry.first) != 0)) {
      return false;
    }
  }
  for (const pose_graph::BetweenFactor& factor : factors) {
    graph.AddBetweenFactor(factor);
  }

  pose_graph::SolverOptions options;
  options.maximum_update_count = 6;
  options.convergence_tolerance = 0.01;
  pose_graph::PoseGraphSolver solver;
  const pose_graph::UpdateResult update = solver.Update(graph, options);
  if (!update.Succeeded()) {
    LogFactorScale(factors, 0, sequential_factor_count, "sequential");
    LogFactorScale(factors, sequential_factor_count, factors.size(),
                   "global");
    LOG(WARNING) << "OptimizePoseGraph could not solve graph with GTSAM: "
                 << update.diagnostic_message;
    return false;
  }
  const pose_graph::SolverResult result = solver.CalculateEstimate();
  if (!result.Succeeded()) {
    LOG(WARNING) << "OptimizePoseGraph could not estimate graph with GTSAM: "
                 << result.diagnostic_message;
    return false;
  }
  for (const auto& entry : initial_poses) {
    const auto optimized_pose = result.optimized_poses.find(entry.first);
    if (optimized_pose == result.optimized_poses.end() ||
        !optimized_pose->second.translation().allFinite() ||
        !optimized_pose->second.rotation().coeffs().allFinite() ||
        optimized_pose->second.rotation().norm() < 1e-12) {
      LOG(ERROR) << "OptimizePoseGraph received an incomplete pose solution";
      return false;
    }
  }

  for (size_t session = 0; session < session_ids.size(); ++session) {
    const int session_id = session_ids[session];
    auto* scans = (*multimap_scans)[static_cast<size_t>(session_id)];
    for (size_t i = 0; i < scans->size(); ++i) {
      const int node = stepsizes[session] + static_cast<int>(i);
      if (!(*scans)[i]) {
        continue;
      }
      const auto optimized = result.optimized_poses.find(node);
      (*scans)[i]->SetState(optimized->second.rotation().toRotationMatrix(),
                            optimized->second.translation());
    }

    if (multimap_keyframes == nullptr ||
        session_id >= static_cast<int>(multimap_keyframes->size())) {
      continue;
    }
    auto* keyframes = (*multimap_keyframes)[static_cast<size_t>(session_id)];
    if (keyframes == nullptr) {
      continue;
    }
    for (Keyframe* keyframe : *keyframes) {
      if (keyframe == nullptr || keyframe->keyframe_id < 0 ||
          keyframe->keyframe_id >= static_cast<int>(scans->size()) ||
          !(*scans)[static_cast<size_t>(keyframe->keyframe_id)]) {
        continue;
      }
      keyframe->inertial_state =
          (*scans)[static_cast<size_t>(keyframe->keyframe_id)]->x;
    }
  }

  LOG(INFO) << "OptimizePoseGraph sessions=" << session_ids.size()
            << " poses=" << stepsizes.back() << " factors=" << factors.size()
            << " gba=" << inject_gba << " cost=" << result.final_cost
            << " updates=" << update.update_count;
  return true;
}

}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas

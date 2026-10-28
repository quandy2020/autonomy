/*
 * Copyright 2026 The Openbot Authors
 */
#include "backend/global_mapping/gtsam_pgo.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include "glog/logging.h"
#include "optimization/voxel/gtsam_optimizer.hpp"

namespace argus {
namespace vxslam {
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

bool AddLoopEdges(const PgoEdges& batch, const std::vector<int>& session_ids,
                  const std::vector<int>& stepsizes,
                  backend::OptimizationProblem* problem) {
  if (problem == nullptr || stepsizes.size() != session_ids.size() + 1) {
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
          from == to || problem->poses.count(from) == 0 ||
          problem->poses.count(to) == 0 || !edge.rots[i].allFinite() ||
          !edge.tras[i].allFinite() || !edge.covs[i].allFinite() ||
          edge.covs[i].minCoeff() < 0.0) {
        return false;
      }
      backend::PoseEdge factor;
      factor.from_id = from;
      factor.to_id = to;
      factor.T_i_j =
          transform::Rigid3d(edge.tras[i], Eigen::Quaterniond(edge.rots[i]));
      factor.variances = edge.covs[i].cwiseMax(1e-9);
      factor.use_variances = true;
      problem->edges.push_back(factor);
    }
  }
  return true;
}

}  // namespace

bool OptimizeIsam2(std::vector<std::vector<ScanPosePtr>*>* multimap_scans,
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

  backend::OptimizationProblem problem;
  problem.max_iterations = 5;
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
      problem.poses.emplace(
          node, transform::Rigid3d(scan->x.p, Eigen::Quaterniond(scan->x.R)));
      if (i == 0 || !scans[i - 1]) {
        continue;
      }
      const ImuState& from = scans[i - 1]->x;
      const ImuState& to = scan->x;
      backend::PoseEdge factor;
      factor.from_id = node - 1;
      factor.to_id = node;
      factor.T_i_j =
          transform::Rigid3d(from.R.transpose() * (to.p - from.p),
                             Eigen::Quaterniond(from.R.transpose() * to.R));
      factor.variances = scans[i - 1]->v6;
      if (!factor.variances.allFinite() || factor.variances.minCoeff() <= 0.0) {
        factor.variances = Eigen::Matrix<double, 6, 1>::Constant(1e-4);
      }
      factor.use_variances = true;
      problem.edges.push_back(factor);
    }
  }

  if (!AddLoopEdges(lp_edges, session_ids, stepsizes, &problem) ||
      (inject_gba &&
       (!AddLoopEdges(gba_edges1, session_ids, stepsizes, &problem) ||
        !AddLoopEdges(gba_edges2, session_ids, stepsizes, &problem)))) {
    LOG(ERROR) << "OptimizeIsam2 rejected malformed PGO edge data";
    return false;
  }

  backend::GtsamOptimizer optimizer;
  const backend::OptimizationResult result = optimizer.Solve(problem);
  if (!result.ok) {
    const char* gtsam_available = "no";
#if defined(ARGUS_HAS_GTSAM) && ARGUS_HAS_GTSAM
    gtsam_available = "yes";
#endif
    LOG(WARNING) << "OptimizeIsam2 could not solve graph (GTSAM available="
                 << gtsam_available << ')';
    return false;
  }

  for (size_t session = 0; session < session_ids.size(); ++session) {
    const int session_id = session_ids[session];
    auto* scans = (*multimap_scans)[static_cast<size_t>(session_id)];
    for (size_t i = 0; i < scans->size(); ++i) {
      const int node = stepsizes[session] + static_cast<int>(i);
      if (!(*scans)[i]) {
        continue;
      }
      const auto optimized = result.poses.find(node);
      if (optimized == result.poses.end()) {
        continue;
      }
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
      if (keyframe == nullptr || keyframe->id < 0 ||
          keyframe->id >= static_cast<int>(scans->size()) ||
          !(*scans)[static_cast<size_t>(keyframe->id)]) {
        continue;
      }
      keyframe->x0 = (*scans)[static_cast<size_t>(keyframe->id)]->x;
    }
  }

  LOG(INFO) << "OptimizeIsam2 sessions=" << session_ids.size()
            << " poses=" << stepsizes.back()
            << " factors=" << problem.edges.size() << " gba=" << inject_gba
            << " cost=" << result.root_mean_cost;
  return true;
}

}  // namespace vxslam
}  // namespace argus

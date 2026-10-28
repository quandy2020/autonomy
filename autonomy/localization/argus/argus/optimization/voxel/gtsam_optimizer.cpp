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

#include "optimization/voxel/gtsam_optimizer.hpp"

namespace argus {
namespace vxslam {
namespace backend {

OptimizationResult GtsamOptimizer::Solve(const OptimizationProblem& problem) {
  OptimizationResult result;
#if defined(ARGUS_HAS_GTSAM) && ARGUS_HAS_GTSAM
  std::vector<pose_graph_detail::RelativeEdge> edges;
  edges.reserve(problem.edges.size());
  for (const PoseEdge& e : problem.edges) {
    edges.push_back(pose_graph_detail::RelativeEdge{e.from_id, e.to_id, e.T_i_j,
                                                    e.information, e.variances,
                                                    e.use_variances});
  }
  result.ok = pose_graph_detail::OptimizeWithGtsam(
      problem.poses, edges, problem.max_iterations, &result.poses,
      &result.root_mean_cost, &result.iterations);
#else
  (void)problem;
  result.ok = false;
#endif
  return result;
}

}  // namespace backend
}  // namespace vxslam
}  // namespace argus

#if defined(ARGUS_HAS_GTSAM) && ARGUS_HAS_GTSAM

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <queue>
#include <unordered_set>
#include <vector>

namespace argus {
namespace vxslam {
namespace backend {
namespace pose_graph_detail {
namespace {

gtsam::Pose3 ToGtsam(const transform::Rigid3d& T) {
  const Eigen::Matrix3d R = T.rotation().toRotationMatrix();
  const Eigen::Vector3d t = T.translation();
  return gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(t.x(), t.y(), t.z()));
}

transform::Rigid3d FromGtsam(const gtsam::Pose3& P) {
  const Eigen::Matrix3d R = P.rotation().matrix();
  const gtsam::Point3 t = P.translation();
  return transform::Rigid3d(Eigen::Vector3d(t.x(), t.y(), t.z()),
                            Eigen::Quaterniond(R));
}

}  // namespace

bool OptimizeWithGtsam(
    const std::unordered_map<int, transform::Rigid3d>& poses_in,
    const std::vector<RelativeEdge>& edges, int max_iterations,
    std::unordered_map<int, transform::Rigid3d>* poses_out,
    double* root_mean_cost, int* iterations) {
  if (poses_out == nullptr || poses_in.empty() || edges.empty() ||
      max_iterations < 0) {
    return false;
  }

  std::vector<int> ids;
  ids.reserve(poses_in.size());
  for (const auto& kv : poses_in) {
    ids.push_back(kv.first);
  }
  std::sort(ids.begin(), ids.end());

  try {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    for (int id : ids) {
      const auto& pose = poses_in.at(id);
      if (!pose.translation().allFinite() ||
          !pose.rotation().coeffs().allFinite() ||
          pose.rotation().norm() < 1e-12) {
        return false;
      }
      values.insert(id, ToGtsam(pose));
    }

    // Each disconnected component has its own gauge freedom. Anchor one
    // node per component so a sparse or partial loop graph stays solvable.
    std::unordered_map<int, std::vector<int>> adjacency;
    std::vector<const RelativeEdge*> active_edges;
    active_edges.reserve(edges.size());
    for (const RelativeEdge& e : edges) {
      if (e.from_id == e.to_id || !values.exists(e.from_id) ||
          !values.exists(e.to_id) || !std::isfinite(e.information) ||
          (!e.use_variances && e.information <= 0.0) ||
          (e.use_variances &&
           (!e.variances.allFinite() || e.variances.minCoeff() < 0.0)) ||
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
      return false;
    }

    const auto prior_noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9).finished());
    std::unordered_set<int> visited;
    for (int root : ids) {
      if (!visited.insert(root).second) {
        continue;
      }
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(
          root, values.at<gtsam::Pose3>(root), prior_noise));
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

    gtsam::NonlinearFactorGraph edge_graph;
    for (const RelativeEdge* edge : active_edges) {
      gtsam::SharedNoiseModel noise;
      if (edge->use_variances) {
        const gtsam::Vector6 variances = edge->variances.cwiseMax(1e-9);
        noise = gtsam::noiseModel::Diagonal::Variances(variances);
      } else {
        const double variance = 1.0 / std::max(1e-12, edge->information);
        noise = gtsam::noiseModel::Isotropic::Variance(6, variance);
      }
      const gtsam::BetweenFactor<gtsam::Pose3> factor(
          edge->from_id, edge->to_id, ToGtsam(edge->T_i_j), noise);
      graph.add(factor);
      edge_graph.add(factor);
    }

    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    gtsam::ISAM2 isam(parameters);
    isam.update(graph, values);
    int updates = 1;
    for (int i = 0; i < max_iterations; ++i) {
      isam.update();
      ++updates;
    }
    const gtsam::Values estimate = isam.calculateEstimate();
    std::unordered_map<int, transform::Rigid3d> candidate;
    candidate.reserve(ids.size());
    for (int id : ids) {
      const transform::Rigid3d pose = FromGtsam(estimate.at<gtsam::Pose3>(id));
      if (!pose.translation().allFinite() ||
          !pose.rotation().coeffs().allFinite()) {
        return false;
      }
      candidate.emplace(id, pose);
    }

    const double error = edge_graph.error(estimate);
    if (!std::isfinite(error)) {
      return false;
    }
    poses_out->swap(candidate);
    if (root_mean_cost != nullptr) {
      *root_mean_cost = std::sqrt(std::max(0.0, error) /
                                  static_cast<double>(active_edges.size()));
    }
    if (iterations != nullptr) {
      *iterations = updates;
    }
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

}  // namespace pose_graph_detail
}  // namespace backend
}  // namespace vxslam
}  // namespace argus

#endif  // ARGUS_HAS_GTSAM

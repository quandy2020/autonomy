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

#include "optimization/g2o_graph_solver.hpp"

#include <cmath>

#if defined(__has_include)
#if __has_include( \
    <g2o/core/sparse_optimizer.h>) && __has_include(<g2o/types/slam3d/edge_se3.h>)
#define ARGUS_HAS_G2O 1
#endif
#endif

#ifdef ARGUS_HAS_G2O
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#endif

namespace argus {
namespace optimization {
namespace {

#ifdef ARGUS_HAS_G2O
Eigen::Isometry3d ToIsometry(const transform::Rigid3d& pose) {
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.linear() = pose.rotation().toRotationMatrix();
  isometry.translation() = pose.translation();
  return isometry;
}

transform::Rigid3d FromIsometry(const Eigen::Isometry3d& pose) {
  return transform::Rigid3d(pose.translation(),
                            Eigen::Quaterniond(pose.linear()));
}

Eigen::Matrix<double, 6, 6> Information(double rotation_weight,
                                        double translation_weight) {
  Eigen::Matrix<double, 6, 6> information =
      Eigen::Matrix<double, 6, 6>::Identity();
  information.topLeftCorner<3, 3>() *= translation_weight;
  information.bottomRightCorner<3, 3>() *= rotation_weight;
  return information;
}

class G2oGraphSolver : public GraphSolver {
 public:
  const char* name() const override { return "g2o"; }

  bool Optimize(FactorGraph* graph) override {
    if (graph == nullptr) {
      return false;
    }
    for (const BetweenFactor& between : graph->betweens()) {
      if (between.similarity) {
        return false;
      }
    }
    using BlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>>;
    auto linear =
        std::make_unique<g2o::LinearSolverEigen<BlockSolver::PoseMatrixType>>();
    auto solver = std::make_unique<g2o::OptimizationAlgorithmLevenberg>(
        std::make_unique<BlockSolver>(std::move(linear)));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver.release());
    for (const auto& entry : graph->poses()) {
      auto* vertex = new g2o::VertexSE3();
      vertex->setId(static_cast<int>(entry.first));
      vertex->setEstimate(ToIsometry(entry.second.pose));
      vertex->setFixed(entry.second.fixed);
      optimizer.addVertex(vertex);
    }
    int anchor_id = -1;
    for (const PriorFactor& prior : graph->priors()) {
      if (!graph->Contains(prior.id)) {
        return false;
      }
      auto* anchor = new g2o::VertexSE3();
      anchor->setId(anchor_id);
      --anchor_id;
      anchor->setEstimate(ToIsometry(prior.pose));
      anchor->setFixed(true);
      optimizer.addVertex(anchor);
      auto* edge = new g2o::EdgeSE3();
      edge->setVertex(0, anchor);
      edge->setVertex(1, optimizer.vertex(static_cast<int>(prior.id)));
      edge->setMeasurement(Eigen::Isometry3d::Identity());
      edge->setInformation(
          Information(prior.rotation_weight, prior.translation_weight));
      optimizer.addEdge(edge);
    }
    for (const BetweenFactor& between : graph->betweens()) {
      if (!graph->Contains(between.from) || !graph->Contains(between.to)) {
        return false;
      }
      auto* edge = new g2o::EdgeSE3();
      edge->setVertex(0, optimizer.vertex(static_cast<int>(between.from)));
      edge->setVertex(1, optimizer.vertex(static_cast<int>(between.to)));
      edge->setMeasurement(ToIsometry(between.relative));
      edge->setInformation(
          Information(between.rotation_weight, between.translation_weight));
      optimizer.addEdge(edge);
    }
    optimizer.initializeOptimization();
    optimizer.optimize(20);
    for (const auto& entry : graph->poses()) {
      if (entry.second.fixed) {
        continue;
      }
      const auto* vertex = dynamic_cast<g2o::VertexSE3*>(
          optimizer.vertex(static_cast<int>(entry.first)));
      if (vertex == nullptr ||
          !graph->SetPose(entry.first, FromIsometry(vertex->estimate()))) {
        return false;
      }
    }
    return true;
  }
};
#endif

}  // namespace

bool G2oSolverAvailable() {
#ifdef ARGUS_HAS_G2O
  return true;
#else
  return false;
#endif
}

std::unique_ptr<GraphSolver> CreateG2oSolver() {
#ifdef ARGUS_HAS_G2O
  return std::make_unique<G2oGraphSolver>();
#else
  return nullptr;
#endif
}

}  // namespace optimization
}  // namespace argus

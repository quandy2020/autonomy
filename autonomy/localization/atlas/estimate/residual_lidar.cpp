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

#include "autonomy/localization/atlas/estimate/residual_lidar.hpp"

#include "autonomy/localization/atlas/data/keyframe.hpp"
#include "autonomy/localization/atlas/util/converter.hpp"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include "autolink/common/log.hpp"

namespace autonomy::localization::atlas {
namespace estimate {

int ApplyLidarPointPlaneRefine(data::keyframe* keyfrm,
                               const LidarFactorBatch& batch,
                               int iterations) {
    if (!keyfrm || batch.point_planes.empty() || iterations <= 0) {
        return 0;
    }

    using BlockSolverType = ::g2o::BlockSolver_6_3;
    using LinearSolverType =
        ::g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

    auto linear_solver = std::make_unique<LinearSolverType>();
    auto block_solver =
        std::make_unique<BlockSolverType>(std::move(linear_solver));
    auto* algorithm =
        new ::g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    ::g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(algorithm);
    optimizer.setVerbose(false);

    auto* vtx = new optimize::internal::se3::shot_vertex();
    vtx->setId(0);
    vtx->setEstimate(util::converter::to_g2o_SE3(keyfrm->get_pose_cw()));
    vtx->setFixed(false);
    optimizer.addVertex(vtx);

    int edge_id = 0;
    for (const auto& r : batch.point_planes) {
        auto* edge = new PointPlanePoseEdge();
        edge->setId(edge_id++);
        edge->setVertex(0, vtx);
        edge->set_point_body(r.point_body);
        Vec4_t meas;
        meas << r.normal_world.x(), r.normal_world.y(), r.normal_world.z(), r.d;
        edge->setMeasurement(meas);
        const double w = std::max(1e-6, r.weight);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * w);
        optimizer.addEdge(edge);
    }

    if (edge_id == 0) {
        return 0;
    }

    optimizer.initializeOptimization();
    optimizer.optimize(iterations);

    keyfrm->set_pose_cw(util::converter::to_eigen_mat(vtx->estimate()));
    AINFO << "ApplyLidarPointPlaneRefine: edges=" << edge_id
          << " chi2=" << optimizer.activeChi2();
    return edge_id;
}

}  // namespace estimate
}  // namespace autonomy::localization::atlas

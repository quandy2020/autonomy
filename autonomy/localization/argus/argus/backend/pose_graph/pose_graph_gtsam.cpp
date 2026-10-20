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

#include "argus/backend/pose_graph/pose_graph_internal.hpp"

#if defined(ARGUS_HAS_GTSAM) && ARGUS_HAS_GTSAM

#include <algorithm>
#include <cmath>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace argus {
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
    const std::vector<CeresEdge>& edges, int max_iterations,
    std::unordered_map<int, transform::Rigid3d>* poses_out, double* cost_rms,
    int* iterations) {
    if (poses_out == nullptr || poses_in.size() < 2 || edges.empty()) {
        return false;
    }

    std::vector<int> ids;
    ids.reserve(poses_in.size());
    for (const auto& kv : poses_in) {
        ids.push_back(kv.first);
    }
    std::sort(ids.begin(), ids.end());

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    for (int id : ids) {
        values.insert(id, ToGtsam(poses_in.at(id)));
    }

    // Gauge: strong prior on lowest id (same role as Ceres SetConstant).
    {
        const auto prior_noise =
            gtsam::noiseModel::Diagonal::Precisions(
                (gtsam::Vector(6) << 1e8, 1e8, 1e8, 1e8, 1e8, 1e8).finished());
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(ids.front(),
                                                   values.at<gtsam::Pose3>(ids.front()),
                                                   prior_noise));
    }

    int active = 0;
    for (const CeresEdge& e : edges) {
        if (!values.exists(e.from_id) || !values.exists(e.to_id)) {
            continue;
        }
        const double prec = std::max(1e-6, e.information);
        const auto noise = gtsam::noiseModel::Isotropic::Precision(6, prec);
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            e.from_id, e.to_id, ToGtsam(e.T_i_j), noise));
        ++active;
    }
    if (active == 0) {
        return false;
    }

    gtsam::LevenbergMarquardtParams params;
    params.maxIterations = std::max(1, max_iterations);
    params.verbosity = gtsam::NonlinearOptimizerParams::SILENT;
    gtsam::LevenbergMarquardtOptimizer opt(graph, values, params);
    const gtsam::Values result = opt.optimize();

    poses_out->clear();
    for (int id : ids) {
        (*poses_out)[id] = FromGtsam(result.at<gtsam::Pose3>(id));
    }
    if (cost_rms != nullptr) {
        const double err = graph.error(result);
        *cost_rms =
            (active > 0) ? std::sqrt(std::max(0.0, err) /
                                     static_cast<double>(active))
                         : 0.0;
    }
    if (iterations != nullptr) {
        *iterations = static_cast<int>(opt.iterations());
    }
    return true;
}

}  // namespace pose_graph_detail
}  // namespace backend
}  // namespace argus

#endif  // ARGUS_HAS_GTSAM

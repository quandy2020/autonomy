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

#if defined(ARGUS_HAS_CERES) && ARGUS_HAS_CERES

#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_map>
#include <vector>

#include <ceres/ceres.h>
#include <ceres/manifold.h>
#include <Eigen/Geometry>

#include "argus/common/math/so3.hpp"

namespace argus {
namespace backend {
namespace pose_graph_detail {
namespace {

struct RelativePoseCost {
    RelativePoseCost(const Eigen::Quaterniond& q_meas,
                     const Eigen::Vector3d& t_meas, double sqrt_info)
        : q_meas_(q_meas.normalized()),
          t_meas_(t_meas),
          sqrt_info_(sqrt_info) {}

    template <typename T>
    bool operator()(const T* const q_i, const T* const t_i, const T* const q_j,
                    const T* const t_j, T* residuals) const {
        const Eigen::Quaternion<T> qi(q_i[3], q_i[0], q_i[1], q_i[2]);
        const Eigen::Quaternion<T> qj(q_j[3], q_j[0], q_j[1], q_j[2]);
        const Eigen::Matrix<T, 3, 1> ti(t_i[0], t_i[1], t_i[2]);
        const Eigen::Matrix<T, 3, 1> tj(t_j[0], t_j[1], t_j[2]);

        const Eigen::Quaternion<T> q_meas = q_meas_.cast<T>();
        const Eigen::Matrix<T, 3, 1> t_meas = t_meas_.cast<T>();

        // Measured: T_i_j. Estimated: T_i^{-1} T_j.
        const Eigen::Quaternion<T> q_est = qi.conjugate() * qj;
        const Eigen::Matrix<T, 3, 1> t_est = qi.conjugate() * (tj - ti);

        const Eigen::Quaternion<T> q_err = q_meas.conjugate() * q_est;
        const Eigen::Matrix<T, 3, 1> t_err = q_meas.conjugate() * (t_est - t_meas);

        // Quaternion vector part ≈ ½θ for small error (Ceres style).
        residuals[0] = T(sqrt_info_) * t_err.x();
        residuals[1] = T(sqrt_info_) * t_err.y();
        residuals[2] = T(sqrt_info_) * t_err.z();
        residuals[3] = T(sqrt_info_) * T(2.0) * q_err.x();
        residuals[4] = T(sqrt_info_) * T(2.0) * q_err.y();
        residuals[5] = T(sqrt_info_) * T(2.0) * q_err.z();
        return true;
    }

    static ceres::CostFunction* Create(const transform::Rigid3d& T_i_j,
                                       double information) {
        const double sqrt_info = std::sqrt(std::max(information, 1e-9));
        return new ceres::AutoDiffCostFunction<RelativePoseCost, 6, 4, 3, 4, 3>(
            new RelativePoseCost(T_i_j.rotation(), T_i_j.translation(),
                                 sqrt_info));
    }

    Eigen::Quaterniond q_meas_;
    Eigen::Vector3d t_meas_;
    double sqrt_info_ = 1.0;
};

void PoseToBlocks(const transform::Rigid3d& T, double* q, double* t) {
    const Eigen::Quaterniond qq = T.rotation().normalized();
    q[0] = qq.x();
    q[1] = qq.y();
    q[2] = qq.z();
    q[3] = qq.w();
    t[0] = T.translation().x();
    t[1] = T.translation().y();
    t[2] = T.translation().z();
}

transform::Rigid3d BlocksToPose(const double* q, const double* t) {
    Eigen::Quaterniond qq(q[3], q[0], q[1], q[2]);
    qq.normalize();
    return transform::Rigid3d(Eigen::Vector3d(t[0], t[1], t[2]), qq);
}

}  // namespace

bool OptimizeWithCeres(
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

    std::unordered_map<int, std::array<double, 4>> q_params;
    std::unordered_map<int, std::array<double, 3>> t_params;
    q_params.reserve(ids.size());
    t_params.reserve(ids.size());
    for (int id : ids) {
        double q[4];
        double t[3];
        PoseToBlocks(poses_in.at(id), q, t);
        q_params[id] = {q[0], q[1], q[2], q[3]};
        t_params[id] = {t[0], t[1], t[2]};
    }

    ceres::Problem problem;
    for (int id : ids) {
        problem.AddParameterBlock(q_params[id].data(), 4);
        // Ownership transferred per block (do not share one Manifold*).
        problem.SetManifold(q_params[id].data(),
                            new ceres::EigenQuaternionManifold());
        problem.AddParameterBlock(t_params[id].data(), 3);
    }
    // Gauge: fix lowest id.
    problem.SetParameterBlockConstant(q_params[ids.front()].data());
    problem.SetParameterBlockConstant(t_params[ids.front()].data());

    int active = 0;
    for (const CeresEdge& e : edges) {
        if (!q_params.count(e.from_id) || !q_params.count(e.to_id)) {
            continue;
        }
        problem.AddResidualBlock(
            RelativePoseCost::Create(e.T_i_j, e.information), nullptr,
            q_params[e.from_id].data(), t_params[e.from_id].data(),
            q_params[e.to_id].data(), t_params[e.to_id].data());
        ++active;
    }
    if (active == 0) {
        return false;
    }

    ceres::Solver::Options options;
    options.max_num_iterations = std::max(1, max_iterations);
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    options.logging_type = ceres::SILENT;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    poses_out->clear();
    for (int id : ids) {
        (*poses_out)[id] =
            BlocksToPose(q_params[id].data(), t_params[id].data());
    }
    if (cost_rms != nullptr) {
        *cost_rms = (active > 0)
                        ? std::sqrt(std::max(0.0, summary.final_cost) /
                                    static_cast<double>(active))
                        : 0.0;
    }
    if (iterations != nullptr) {
        *iterations = summary.iterations.size();
    }
    return summary.IsSolutionUsable();
}

}  // namespace pose_graph_detail
}  // namespace backend
}  // namespace argus

#endif  // ARGUS_HAS_CERES

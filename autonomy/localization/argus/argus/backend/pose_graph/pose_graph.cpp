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

#include "argus/backend/pose_graph/pose_graph.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "argus/common/math/so3.hpp"
#include "argus/common/math/transform/transform.hpp"

namespace argus {
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
    const Eigen::Matrix3d R = math::Exp(phi);
    return transform::Rigid3d(xi.head<3>(), Eigen::Quaterniond(R));
}

/** @brief Right-perturb: T ⊕ δ = T * Exp(δ). */
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
        Eigen::Matrix<double, 6, 1> delta =
            Eigen::Matrix<double, 6, 1>::Zero();
        delta(d) = eps;
        const Eigen::Matrix<double, 6, 1> rip =
            RelativeError(RightPlus(Ti, delta), Tj, Tij);
        const Eigen::Matrix<double, 6, 1> rim =
            RelativeError(RightPlus(Ti, -delta), Tj, Tij);
        Ji->col(d) = (rip - rim) / (2.0 * eps);

        const Eigen::Matrix<double, 6, 1> rjp =
            RelativeError(Ti, RightPlus(Tj, delta), Tij);
        const Eigen::Matrix<double, 6, 1> rjm =
            RelativeError(Ti, RightPlus(Tj, -delta), Tij);
        Jj->col(d) = (rjp - rjm) / (2.0 * eps);
    }
}

void AccumulateEdge(int from_id, int to_id, const transform::Rigid3d& T_i_j,
                    double information,
                    const std::unordered_map<int, int>& id_to_index,
                    const std::vector<transform::Rigid3d>& poses, double eps,
                    Eigen::MatrixXd* H, Eigen::VectorXd* b, double* cost) {
    if (!id_to_index.count(from_id) || !id_to_index.count(to_id)) {
        return;
    }
    const int from_index = id_to_index.at(from_id);
    const int to_index = id_to_index.at(to_id);
    const transform::Rigid3d& Ti =
        poses[static_cast<std::size_t>(from_index)];
    const transform::Rigid3d& Tj =
        poses[static_cast<std::size_t>(to_index)];
    const Eigen::Matrix<double, 6, 1> residual =
        RelativeError(Ti, Tj, T_i_j);
    *cost += information * residual.squaredNorm();

    Eigen::Matrix<double, 6, 6> Ji;
    Eigen::Matrix<double, 6, 6> Jj;
    FiniteDiffJacobians(Ti, Tj, T_i_j, eps, &Ji, &Jj);

    const double weight = information;
    H->block<6, 6>(6 * from_index, 6 * from_index) +=
        weight * Ji.transpose() * Ji;
    H->block<6, 6>(6 * to_index, 6 * to_index) +=
        weight * Jj.transpose() * Jj;
    H->block<6, 6>(6 * from_index, 6 * to_index) +=
        weight * Ji.transpose() * Jj;
    H->block<6, 6>(6 * to_index, 6 * from_index) +=
        weight * Jj.transpose() * Ji;
    b->segment<6>(6 * from_index) += weight * Ji.transpose() * residual;
    b->segment<6>(6 * to_index) += weight * Jj.transpose() * residual;
}

void FixGauge(Eigen::MatrixXd* H, Eigen::VectorXd* b) {
    for (int k = 0; k < 6; ++k) {
        H->row(k).setZero();
        H->col(k).setZero();
        (*H)(k, k) = 1.0;
        (*b)(k) = 0.0;
    }
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

void PoseGraph::Optimize(int max_iterations) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (poses_.size() < 2 || edges_.empty()) {
        return;
    }

    std::vector<int> ids;
    ids.reserve(poses_.size());
    for (const auto& kv : poses_) {
        ids.push_back(kv.first);
    }
    std::sort(ids.begin(), ids.end());
    const int node_count = static_cast<int>(ids.size());
    std::unordered_map<int, int> id_to_index;
    for (int k = 0; k < node_count; ++k) {
        id_to_index[ids[static_cast<std::size_t>(k)]] = k;
    }

    std::vector<transform::Rigid3d> poses(static_cast<std::size_t>(node_count));
    for (int k = 0; k < node_count; ++k) {
        poses[static_cast<std::size_t>(k)] =
            poses_[ids[static_cast<std::size_t>(k)]];
    }

    constexpr double kEps = 1e-6;
    double cost = 0.0;

    for (int iter = 0; iter < max_iterations; ++iter) {
        const int dim = 6 * node_count;
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dim, dim);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(dim);
        cost = 0.0;

        for (const Edge& edge : edges_) {
            AccumulateEdge(edge.from_id, edge.to_id, edge.T_i_j,
                           edge.information, id_to_index, poses, kEps, &H, &b,
                           &cost);
        }

        // Gauge: fix first node.
        FixGauge(&H, &b);

        const Eigen::VectorXd dx = H.ldlt().solve(-b);
        if (!dx.allFinite()) {
            break;
        }

        for (int k = 0; k < node_count; ++k) {
            poses[static_cast<std::size_t>(k)] = RightPlus(
                poses[static_cast<std::size_t>(k)], dx.segment<6>(6 * k));
        }
        if (dx.norm() < 1e-6) {
            break;
        }
    }

    int active_edge_count = 0;
    for (const Edge& edge : edges_) {
        if (id_to_index.count(edge.from_id) &&
            id_to_index.count(edge.to_id)) {
            ++active_edge_count;
        }
    }
    last_cost_ =
        (active_edge_count > 0)
            ? std::sqrt(cost / static_cast<double>(active_edge_count))
            : 0.0;

    for (int k = 0; k < node_count; ++k) {
        poses_[ids[static_cast<std::size_t>(k)]] =
            poses[static_cast<std::size_t>(k)];
    }
}

}  // namespace backend
}  // namespace argus

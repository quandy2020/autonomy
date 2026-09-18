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

#include "autonomy/localization/atlas/backend/lidar_pose_graph.hpp"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <algorithm>
#include <cmath>

#include "glog/logging.h"

namespace autonomy::localization::atlas {
namespace backend {

Eigen::Isometry3d LidarPoseGraph::ToIso(const Mat44_t& T) {
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.linear() = T.block<3, 3>(0, 0);
    iso.translation() = T.block<3, 1>(0, 3);
    return iso;
}

Mat44_t LidarPoseGraph::FromIso(const Eigen::Isometry3d& iso) {
    Mat44_t T = Mat44_t::Identity();
    T.block<3, 3>(0, 0) = iso.rotation();
    T.block<3, 1>(0, 3) = iso.translation();
    return T;
}

Eigen::Matrix<double, 6, 6> LidarPoseGraph::MotionInfo() const {
    Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Identity();
    const double st = std::max(1e-6, options_.motion_trans_noise);
    const double sr = std::max(1e-6, options_.motion_rot_noise);
    info.block<3, 3>(0, 0) *= 1.0 / (st * st);
    info.block<3, 3>(3, 3) *= 1.0 / (sr * sr);
    return info;
}

Eigen::Matrix<double, 6, 6> LidarPoseGraph::LoopInfo(double score) const {
    Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Identity();
    const double st = std::max(1e-6, options_.loop_trans_noise);
    const double sr = std::max(1e-6, options_.loop_rot_noise);
    const double w = std::max(0.1, score);
    info.block<3, 3>(0, 0) *= w / (st * st);
    info.block<3, 3>(3, 3) *= w / (sr * sr);
    return info;
}

void LidarPoseGraph::Clear() {
    poses_.clear();
    odom_edges_.clear();
    loops_.clear();
    has_last_ = false;
    last_id_ = std::numeric_limits<std::uint64_t>::max();
}

void LidarPoseGraph::AddKeyframe(std::uint64_t id, const Mat44_t& T_wb) {
    poses_[id] = T_wb;
    if (has_last_) {
        const auto it = poses_.find(last_id_);
        if (it != poses_.end()) {
            EdgeRec e;
            e.id_from = last_id_;
            e.id_to = id;
            e.T_from_to = it->second.inverse() * T_wb;
            e.is_loop = false;
            odom_edges_.push_back(std::move(e));
        }
    }
    last_id_ = id;
    has_last_ = true;
}

void LidarPoseGraph::AddLoop(std::uint64_t id_from, std::uint64_t id_to,
                             const Mat44_t& T_from_to, double score) {
    if (poses_.find(id_from) == poses_.end() ||
        poses_.find(id_to) == poses_.end()) {
        return;
    }
    EdgeRec e;
    e.id_from = id_from;
    e.id_to = id_to;
    e.T_from_to = T_from_to;
    e.score = score;
    e.is_loop = true;
    loops_.push_back(std::move(e));
}

bool LidarPoseGraph::Optimize(int iters) {
    if (poses_.size() < 2 || loops_.empty()) {
        return false;
    }

    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    using BlockSolverType = g2o::BlockSolver_6_3;
    using LinearSolverType =
        g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;
    auto linear_solver =
        autonomy::localization::atlas::make_unique<LinearSolverType>();
    auto block_solver =
        autonomy::localization::atlas::make_unique<BlockSolverType>(
            std::move(linear_solver));
    auto* algorithm =
        new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
    optimizer.setAlgorithm(algorithm);

    std::uint64_t min_id = std::numeric_limits<std::uint64_t>::max();
    for (const auto& kv : poses_) {
        min_id = std::min(min_id, kv.first);
        auto* v = new g2o::VertexSE3();
        v->setId(static_cast<int>(kv.first));
        v->setEstimate(ToIso(kv.second));
        v->setFixed(false);
        optimizer.addVertex(v);
    }
    if (auto* first = optimizer.vertex(static_cast<int>(min_id))) {
        first->setFixed(true);
    }

    const auto motion_info = MotionInfo();
    for (const auto& e : odom_edges_) {
        auto* v0 = optimizer.vertex(static_cast<int>(e.id_from));
        auto* v1 = optimizer.vertex(static_cast<int>(e.id_to));
        if (!v0 || !v1) {
            continue;
        }
        auto* edge = new g2o::EdgeSE3();
        edge->setVertex(0, v0);
        edge->setVertex(1, v1);
        edge->setMeasurement(ToIso(e.T_from_to));
        edge->setInformation(motion_info);
        optimizer.addEdge(edge);
    }

    std::vector<g2o::EdgeSE3*> loop_edges;
    loop_edges.reserve(loops_.size());
    for (const auto& e : loops_) {
        auto* v0 = optimizer.vertex(static_cast<int>(e.id_from));
        auto* v1 = optimizer.vertex(static_cast<int>(e.id_to));
        if (!v0 || !v1) {
            continue;
        }
        auto* edge = new g2o::EdgeSE3();
        edge->setVertex(0, v0);
        edge->setVertex(1, v1);
        edge->setMeasurement(ToIso(e.T_from_to));
        edge->setInformation(LoopInfo(e.score));
        if (options_.use_robust_kernel) {
            auto* rk = new g2o::RobustKernelCauchy();
            rk->setDelta(options_.robust_delta);
            edge->setRobustKernel(rk);
        }
        optimizer.addEdge(edge);
        loop_edges.push_back(edge);
    }

    if (optimizer.edges().empty()) {
        return false;
    }

    optimizer.initializeOptimization();
    optimizer.optimize(std::max(1, iters));

    // Optional outlier demotion + second pass (lightning-style).
    if (options_.use_robust_kernel && !loop_edges.empty()) {
        int outliers = 0;
        for (auto* edge : loop_edges) {
            if (!edge->robustKernel()) {
                continue;
            }
            if (edge->chi2() > edge->robustKernel()->delta()) {
                edge->setLevel(1);
                ++outliers;
            } else {
                edge->setRobustKernel(nullptr);
            }
        }
        if (outliers > 0) {
            optimizer.initializeOptimization(0);
            optimizer.optimize(std::max(1, iters / 2));
        }
        VLOG(1) << "LidarPoseGraph: loop outliers=" << outliers << "/"
                << loop_edges.size();
    }

    for (auto& kv : poses_) {
        auto* v = dynamic_cast<g2o::VertexSE3*>(
            optimizer.vertex(static_cast<int>(kv.first)));
        if (!v) {
            continue;
        }
        kv.second = FromIso(v->estimate());
    }
    return true;
}

bool LidarPoseGraph::GetPose(std::uint64_t id, Mat44_t* T) const {
    if (!T) {
        return false;
    }
    const auto it = poses_.find(id);
    if (it == poses_.end()) {
        return false;
    }
    *T = it->second;
    return true;
}

Mat44_t LidarPoseGraph::GetPoseOr(std::uint64_t id,
                                  const Mat44_t& fallback) const {
    Mat44_t T;
    if (GetPose(id, &T)) {
        return T;
    }
    return fallback;
}

}  // namespace backend
}  // namespace autonomy::localization::atlas

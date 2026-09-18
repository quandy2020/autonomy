/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/localization/atlas/optimize/global_bundle_adjuster_inertial.hpp"

#include "autonomy/localization/atlas/data/keyframe.hpp"
#include "autonomy/localization/atlas/data/map_database.hpp"
#include "autonomy/localization/atlas/optimize/imu_g2o/bias_vertex.hpp"
#include "autonomy/localization/atlas/optimize/imu_g2o/preintegration_edge.hpp"
#include "autonomy/localization/atlas/optimize/imu_g2o/velocity_vertex.hpp"
#include "autonomy/localization/atlas/optimize/internal/se3/shot_vertex.hpp"
#include "autonomy/localization/atlas/util/converter.hpp"

#include <algorithm>
#include <unordered_map>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include "autolink/common/log.hpp"

namespace autonomy::localization::atlas {
namespace optimize {

global_bundle_adjuster_inertial::global_bundle_adjuster_inertial(const imu::config& imu_cfg,
                                                                 unsigned int num_iter)
    : imu_cfg_(imu_cfg), num_iter_(num_iter),
      gravity_(0.0, 0.0, -imu_cfg.gravity_magnitude) {}

bool global_bundle_adjuster_inertial::optimize(data::map_database* map_db,
                                               bool* force_stop_flag) const {
    if (!map_db || !imu_cfg_.enabled) {
        return false;
    }
    auto keyfrms = map_db->get_all_keyframes();
    std::sort(keyfrms.begin(), keyfrms.end(),
              [](const auto& a, const auto& b) { return a->id_ < b->id_; });
    if (keyfrms.size() < 2) {
        return false;
    }

    auto linear_solver = std::make_unique<::g2o::LinearSolverEigen<::g2o::BlockSolverX::PoseMatrixType>>();
    auto block_solver = std::make_unique<::g2o::BlockSolverX>(std::move(linear_solver));
    auto* algorithm = new ::g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
    ::g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(algorithm);

    std::unordered_map<unsigned int, internal::se3::shot_vertex*> pose_vertices;
    std::unordered_map<unsigned int, imu_g2o::velocity_vertex*> vel_vertices;
    std::unordered_map<unsigned int, imu_g2o::bias_vertex*> bias_vertices;

    int vtx_id = 0;
    for (const auto& kf : keyfrms) {
        if (!kf || kf->will_be_erased()) {
            continue;
        }
        auto* v_pose = new internal::se3::shot_vertex();
        v_pose->setId(vtx_id++);
        v_pose->setEstimate(util::converter::to_g2o_SE3(kf->get_pose_cw()));
        v_pose->setFixed(kf->id_ == keyfrms.front()->id_);
        optimizer.addVertex(v_pose);
        pose_vertices[kf->id_] = v_pose;

        auto* v_vel = new imu_g2o::velocity_vertex();
        v_vel->setId(vtx_id++);
        v_vel->setEstimate(kf->get_velocity());
        optimizer.addVertex(v_vel);
        vel_vertices[kf->id_] = v_vel;

        auto* v_bias = new imu_g2o::bias_vertex();
        v_bias->setId(vtx_id++);
        v_bias->setBias(kf->get_imu_bias());
        optimizer.addVertex(v_bias);
        bias_vertices[kf->id_] = v_bias;
    }

    unsigned int num_edges = 0;
    for (const auto& kf : keyfrms) {
        if (!kf) {
            continue;
        }
        const auto preint = kf->get_imu_preintegrator();
        const auto prev = kf->get_imu_prev_keyframe();
        if (!preint || !preint->is_valid() || !prev) {
            continue;
        }
        if (!pose_vertices.count(prev->id_) || !pose_vertices.count(kf->id_)) {
            continue;
        }
        auto* edge = new imu_g2o::preintegration_edge();
        edge->setId(vtx_id++);
        edge->setVertex(0, pose_vertices[prev->id_]);
        edge->setVertex(1, vel_vertices[prev->id_]);
        edge->setVertex(2, bias_vertices[prev->id_]);
        edge->setVertex(3, pose_vertices[kf->id_]);
        edge->setVertex(4, vel_vertices[kf->id_]);
        edge->setMeasurement(preint);
        edge->setGravity(gravity_);
        edge->setExtrinsic(imu_cfg_.T_c_b());
        edge->setInformationFromPreintegrator();
        optimizer.addEdge(edge);

        auto* walk = new imu_g2o::bias_walk_edge();
        walk->setId(vtx_id++);
        walk->setVertex(0, bias_vertices[prev->id_]);
        walk->setVertex(1, bias_vertices[kf->id_]);
        walk->setWalkInformation(imu_cfg_.random_walk_acc, imu_cfg_.random_walk_gyro, preint->delta_t());
        optimizer.addEdge(walk);
        ++num_edges;
    }

    if (num_edges == 0) {
        return false;
    }
    if (force_stop_flag) {
        optimizer.setForceStopFlag(force_stop_flag);
    }
    optimizer.initializeOptimization();
    optimizer.optimize(static_cast<int>(num_iter_));

    for (const auto& kf : keyfrms) {
        if (!kf || kf->will_be_erased() || !pose_vertices.count(kf->id_)) {
            continue;
        }
        kf->set_pose_cw(util::converter::to_eigen_mat(pose_vertices[kf->id_]->estimate()));
        kf->set_velocity(vel_vertices[kf->id_]->estimate());
        const auto new_bias = bias_vertices[kf->id_]->bias();
        kf->set_imu_bias(new_bias);
        if (auto next = kf->get_imu_next_keyframe()) {
            if (auto pre = next->get_imu_preintegrator()) {
                pre->update_bias(new_bias);
            }
        }
    }
    AINFO << "global_bundle_adjuster_inertial: optimized " << num_edges << " IMU edges";
    return true;
}

}  // namespace optimize
}  // namespace autonomy::localization::atlas

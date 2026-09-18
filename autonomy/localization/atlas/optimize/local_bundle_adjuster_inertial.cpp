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

#include "autonomy/localization/atlas/optimize/local_bundle_adjuster_inertial.hpp"

#include "autonomy/localization/atlas/data/keyframe.hpp"
#include "autonomy/localization/atlas/data/map_database.hpp"
#include "autonomy/localization/atlas/optimize/imu_g2o/bias_vertex.hpp"
#include "autonomy/localization/atlas/optimize/imu_g2o/preintegration_edge.hpp"
#include "autonomy/localization/atlas/optimize/imu_g2o/velocity_vertex.hpp"
#include "autonomy/localization/atlas/optimize/internal/se3/shot_vertex.hpp"
#include "autonomy/localization/atlas/util/converter.hpp"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include "autolink/common/log.hpp"

namespace autonomy::localization::atlas {
namespace optimize {

local_bundle_adjuster_inertial::local_bundle_adjuster_inertial(const YAML::Node& yaml_node,
                                                             const imu::config& imu_cfg)
    : visual_ba_(yaml_node),
      imu_cfg_(imu_cfg),
      num_first_iter_(yaml_node["num_first_iter"].as<unsigned int>(5)),
      num_second_iter_(yaml_node["num_second_iter"].as<unsigned int>(10)) {}

void local_bundle_adjuster_inertial::set_gravity(const Vec3_t& g) {
    gravity_ = g;
}

void local_bundle_adjuster_inertial::optimize(data::map_database* map_db,
                                             const std::shared_ptr<data::keyframe>& curr_keyfrm,
                                             bool* const force_stop_flag) const {
    if (!imu_cfg_.enabled || !inertial_ready_) {
        visual_ba_.optimize(map_db, curr_keyfrm, force_stop_flag);
        return;
    }
    // Visual local BA first (landmarks + poses), then joint inertial refine on window.
    visual_ba_.optimize(map_db, curr_keyfrm, force_stop_flag);
    if (force_stop_flag && *force_stop_flag) {
        return;
    }
    optimize_inertial(map_db, curr_keyfrm, force_stop_flag);
}

void local_bundle_adjuster_inertial::optimize_inertial(
    data::map_database* map_db,
    const std::shared_ptr<data::keyframe>& curr_keyfrm,
    bool* const force_stop_flag) const {
    if (!curr_keyfrm) {
        return;
    }

    // Collect local window + temporal IMU neighbors.
    std::unordered_map<unsigned int, std::shared_ptr<data::keyframe>> local_map;
    auto covis = curr_keyfrm->graph_node_->get_covisibilities();
    covis.push_back(curr_keyfrm);
    for (const auto& kf : covis) {
        if (kf && !kf->will_be_erased()) {
            local_map[kf->id_] = kf;
            if (auto prev = kf->get_imu_prev_keyframe()) {
                local_map[prev->id_] = prev;
            }
            if (auto next = kf->get_imu_next_keyframe()) {
                local_map[next->id_] = next;
            }
        }
    }
    std::vector<std::shared_ptr<data::keyframe>> local_keyfrms;
    local_keyfrms.reserve(local_map.size());
    for (auto& kv : local_map) {
        local_keyfrms.push_back(kv.second);
    }
    std::sort(local_keyfrms.begin(), local_keyfrms.end(),
              [](const auto& a, const auto& b) { return a->id_ < b->id_; });
    if (local_keyfrms.size() < 2) {
        return;
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
    const unsigned int oldest_id = local_keyfrms.front()->id_;
    for (const auto& kf : local_keyfrms) {
        if (!kf || kf->will_be_erased()) {
            continue;
        }
        auto* v_pose = new internal::se3::shot_vertex();
        v_pose->setId(vtx_id++);
        v_pose->setEstimate(util::converter::to_g2o_SE3(kf->get_pose_cw()));
        v_pose->setFixed(kf->id_ == oldest_id);
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

    unsigned int num_imu_edges = 0;
    for (const auto& kf : local_keyfrms) {
        if (!kf) {
            continue;
        }
        const auto preint = kf->get_imu_preintegrator();
        if (!preint || !preint->is_valid()) {
            continue;
        }
        const auto parent = kf->get_imu_prev_keyframe();
        if (!parent) {
            continue;
        }
        auto it_i_pose = pose_vertices.find(parent->id_);
        auto it_j_pose = pose_vertices.find(kf->id_);
        if (it_i_pose == pose_vertices.end() || it_j_pose == pose_vertices.end()) {
            continue;
        }

        auto* edge = new imu_g2o::preintegration_edge();
        edge->setId(vtx_id++);
        edge->setVertex(0, it_i_pose->second);
        edge->setVertex(1, vel_vertices[parent->id_]);
        edge->setVertex(2, bias_vertices[parent->id_]);
        edge->setVertex(3, it_j_pose->second);
        edge->setVertex(4, vel_vertices[kf->id_]);
        edge->setMeasurement(preint);
        edge->setGravity(gravity_);
        edge->setExtrinsic(imu_cfg_.T_c_b());
        edge->setInformationFromPreintegrator();
        auto* rk = new ::g2o::RobustKernelHuber();
        rk->setDelta(std::sqrt(16.92));
        edge->setRobustKernel(rk);
        optimizer.addEdge(edge);

        auto* walk = new imu_g2o::bias_walk_edge();
        walk->setId(vtx_id++);
        walk->setVertex(0, bias_vertices[parent->id_]);
        walk->setVertex(1, bias_vertices[kf->id_]);
        walk->setWalkInformation(imu_cfg_.random_walk_acc, imu_cfg_.random_walk_gyro, preint->delta_t());
        optimizer.addEdge(walk);
        ++num_imu_edges;
    }

    if (num_imu_edges == 0) {
        ADEBUG << "local_bundle_adjuster_inertial: no temporal IMU edges";
        return;
    }

    if (force_stop_flag) {
        optimizer.setForceStopFlag(force_stop_flag);
    }
    optimizer.initializeOptimization();
    optimizer.optimize(static_cast<int>(num_second_iter_));

    for (const auto& kf : local_keyfrms) {
        if (!kf || kf->will_be_erased()) {
            continue;
        }
        auto it_pose = pose_vertices.find(kf->id_);
        if (it_pose == pose_vertices.end()) {
            continue;
        }
        kf->set_pose_cw(util::converter::to_eigen_mat(it_pose->second->estimate()));
        kf->set_velocity(vel_vertices[kf->id_]->estimate());
        const auto new_bias = bias_vertices[kf->id_]->bias();
        kf->set_imu_bias(new_bias);
        // Relinearize preintegration on the outgoing temporal edge (this KF -> next).
        if (auto next = kf->get_imu_next_keyframe()) {
            if (auto pre = next->get_imu_preintegrator()) {
                pre->update_bias(new_bias);
            }
        }
    }

    (void)map_db;
    (void)num_first_iter_;
}

}  // namespace optimize
}  // namespace autonomy::localization::atlas

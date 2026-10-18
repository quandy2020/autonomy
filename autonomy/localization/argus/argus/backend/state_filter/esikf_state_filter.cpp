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

#include "argus/backend/state_filter/esikf_state_filter.hpp"

#include <cmath>

#include "argus/backend/state_filter/esikf_update_options.hpp"
#include "argus/common/math/so3.hpp"
#include "argus/frontend/types.hpp"
#include "argus/fusion/anisotropic_covariance.hpp"
#include "argus/fusion/intensity_assist.hpp"

namespace argus {
namespace backend {
namespace {

using frontend::PointWithVar;
using map::calcBodyCov;

void BuildBodyCovAndCrossMat(map::VoxelMapManager* map) {
    auto& frame = map->frame();
    auto& cross_mat_list = frame.cross_mat_list;
    auto& body_cov_list = frame.body_cov_list;
    auto& feats_down_body = frame.feats_down_body;
    const auto& config = map->config_setting_;
    const auto& extR = map->extR_;
    const auto& extT = map->extT_;

    const std::size_t body_point_count = feats_down_body->size();
    cross_mat_list.resize(body_point_count);
    body_cov_list.resize(body_point_count);
    frame.pv_list.resize(static_cast<std::size_t>(frame.feats_down_size));

    for (std::size_t i = 0; i < body_point_count; ++i) {
        Eigen::Vector3d point_body(feats_down_body->points[i].x,
                                   feats_down_body->points[i].y,
                                   feats_down_body->points[i].z);
        if (point_body[2] == 0) {
            point_body[2] = 0.001;
        }
        Eigen::Matrix3d body_cov;
        calcBodyCov(point_body, config.dept_err_, config.beam_err_, body_cov);
        body_cov_list[i] = body_cov;
        point_body = extR * point_body + extT;
        Eigen::Matrix3d point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_body);
        cross_mat_list[i] = point_crossmat;
    }
}

void FillPvListWorldVars(map::VoxelMapManager* map) {
    auto& frame = map->frame();
    auto& state = map->state_;
    auto& world_lidar = frame.world_lidar;
    map->TransformLidar(state.rot_end, state.pos_end, frame.feats_down_body,
                        world_lidar);
    const Eigen::Matrix3d rot_cov = state.cov.block<3, 3>(0, 0);
    const Eigen::Matrix3d trans_cov = state.cov.block<3, 3>(3, 3);
    for (std::size_t i = 0; i < frame.feats_down_body->size(); ++i) {
        PointWithVar& pv = frame.pv_list[i];
        pv.point_b << frame.feats_down_body->points[i].x,
            frame.feats_down_body->points[i].y,
            frame.feats_down_body->points[i].z;
        pv.point_w << world_lidar->points[i].x, world_lidar->points[i].y,
            world_lidar->points[i].z;

        Eigen::Matrix3d cov = frame.body_cov_list[i];
        const Eigen::Matrix3d& point_crossmat = frame.cross_mat_list[i];
        cov = state.rot_end * cov * state.rot_end.transpose() +
              (-point_crossmat) * rot_cov * (-point_crossmat.transpose()) +
              trans_cov;
        pv.var = cov;
        pv.body_var = frame.body_cov_list[i];
    }
}

void BuildMeasurementJacobian(map::VoxelMapManager* map,
                              frontend::StatesGroup* state_propagated) {
    auto& frame = map->frame();
    auto& ptpl_list = frame.ptpl_list;
    auto& effect_feat_num = frame.effect_feat_num;
    auto& state = map->state_;
    const auto& extR = map->extR_;
    const auto& extT = map->extT_;

    effect_feat_num = static_cast<int>(ptpl_list.size());
    if (effect_feat_num == 0) {
        return;
    }

    auto& Hsub = frame.Hsub;
    auto& Hsub_T_R_inv = frame.Hsub_T_R_inv;
    auto& R_inv = frame.R_inv;
    auto& meas_vec = frame.meas_vec;
    Hsub.resize(effect_feat_num, 6);
    Hsub_T_R_inv.resize(6, effect_feat_num);
    R_inv.resize(effect_feat_num);
    meas_vec.resize(effect_feat_num);
    meas_vec.setZero();
    for (int i = 0; i < effect_feat_num; ++i) {
        const auto& ptpl = ptpl_list[i];
        Eigen::Vector3d point_imu = extR * ptpl.point_b_ + extT;
        Eigen::Matrix3d point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_imu);

        const Eigen::Vector3d point_world =
            state_propagated->rot_end * point_imu +
            state_propagated->pos_end;
        Eigen::Matrix<double, 1, 6> J_nq;
        J_nq.block<1, 3>(0, 0) = point_world - ptpl.center_;
        J_nq.block<1, 3>(0, 3) = -ptpl.normal_;

        const Eigen::Matrix3d var =
            state_propagated->rot_end * extR * ptpl.body_cov_ *
            (state_propagated->rot_end * extR).transpose();

        const double sigma_l = J_nq * ptpl.plane_var_ * J_nq.transpose();

        R_inv(i) = 1.0 / (0.001 + sigma_l +
                          ptpl.normal_.transpose() * var * ptpl.normal_);

        const Eigen::Vector3d A(point_crossmat * state.rot_end.transpose() *
                                ptpl.normal_);
        Hsub.row(i) << VEC_FROM_ARRAY(A), ptpl.normal_[0], ptpl.normal_[1],
            ptpl.normal_[2];
        Hsub_T_R_inv.col(i) << A[0] * R_inv(i), A[1] * R_inv(i),
            A[2] * R_inv(i), ptpl.normal_[0] * R_inv(i),
            ptpl.normal_[1] * R_inv(i), ptpl.normal_[2] * R_inv(i);
        meas_vec(i) = -ptpl.dis_to_plane_;
    }
}

// Returns true if LiDAR update should be skipped for this scan.
bool ApplyDegeneracyAndGating(
    map::VoxelMapManager* map, const EsikfUpdateOptions& options,
    Eigen::Matrix<double, frontend::kDimState, frontend::kDimState>& H_T_H,
    Eigen::VectorXd& HTz) {
    auto& state = map->state_;
    auto& feats_down_body = map->frame().feats_down_body;

    const Eigen::Matrix3d fim_position = H_T_H.block<3, 3>(3, 3);
    perception::DegeneracyDetector detector(options.degeneracy);
    const perception::DegeneracyResult degeneracy =
        detector.Analyze(fim_position);
    if (options.degeneracy_result != nullptr) {
        *options.degeneracy_result = degeneracy;
    }

    fusion::ObservationConfidence observation;
    if (options.enable_gating) {
        fusion::ConfidenceGating gate(options.gating);
        observation = gate.Evaluate(degeneracy, options.health,
                                    options.dynamic_ratio_lidar,
                                    options.dynamic_ratio_visual);
    } else {
        observation.is_degenerated = degeneracy.is_degenerated;
        observation.lidar_weak_directions = degeneracy.weak_directions;
        observation.lidar_valid = true;
        observation.lidar_weight = 1.0;
    }
    if (options.observation_confidence != nullptr) {
        *options.observation_confidence = observation;
    }

    if (options.enable_gating && !observation.lidar_valid) {
        return true;
    }

    if (observation.lidar_weight < 1.0 - 1e-12) {
        H_T_H.block<6, 6>(0, 0) *= observation.lidar_weight;
        HTz *= observation.lidar_weight;
    }

    double inflation_alpha = options.degeneracy.inflation_scale;
    if (options.enable_intensity_assist && feats_down_body != nullptr) {
        fusion::IntensityAssistConfig assist_config = options.intensity_assist;
        assist_config.enable = true;
        fusion::IntensityAssist assist(assist_config);
        const fusion::IntensityAssistResult assist_result =
            assist.Evaluate(degeneracy, *feats_down_body);
        if (options.intensity_assist_result != nullptr) {
            *options.intensity_assist_result = assist_result;
        }
        if (assist_result.available) {
            inflation_alpha *= assist_result.inflation_multiplier;
        }
    }

    const bool should_inflate =
        observation.is_degenerated &&
        options.degeneracy.enable_anisotropic_inflation &&
        (options.enable_degeneracy || options.enable_gating);
    if (should_inflate) {
        Eigen::Matrix3d position_cov = state.cov.block<3, 3>(3, 3);
        fusion::AnisotropicCovariance::InflatePositionBlock(
            position_cov, observation.lidar_weak_directions, inflation_alpha);
        state.cov.block<3, 3>(3, 3) = position_cov;
    }
    return false;
}

bool SolveAndApplyEkfStep(
    map::VoxelMapManager* map, frontend::StatesGroup* state_propagated,
    Eigen::Matrix<double, frontend::kDimState, frontend::kDimState>& G,
    Eigen::Matrix<double, frontend::kDimState, frontend::kDimState>& H_T_H,
    const Eigen::VectorXd& HTz,
    Eigen::Matrix<double, frontend::kDimState, frontend::kDimState>& cov_inv,
    bool& cov_inv_ready) {
    auto& state = map->state_;

    if (!cov_inv_ready) {
        cov_inv = state.cov
                      .block<frontend::kDimState, frontend::kDimState>(0, 0)
                      .inverse();
        cov_inv_ready = true;
    }

    Eigen::Matrix<double, frontend::kDimState, frontend::kDimState>&& K_1 =
        (H_T_H.block<frontend::kDimState, frontend::kDimState>(0, 0) + cov_inv)
            .inverse();
    G.block<frontend::kDimState, 6>(0, 0) =
        K_1.block<frontend::kDimState, 6>(0, 0) * H_T_H.block<6, 6>(0, 0);
    const auto error = *state_propagated - state;
    Eigen::Matrix<double, frontend::kDimState, 1> solution =
        K_1.block<frontend::kDimState, 6>(0, 0) * HTz +
        error.block<frontend::kDimState, 1>(0, 0) -
        G.block<frontend::kDimState, 6>(0, 0) * error.block<6, 1>(0, 0);
    state += solution;
    const auto rot_delta = solution.block<3, 1>(0, 0);
    const auto trans_delta = solution.block<3, 1>(3, 0);
    return (rot_delta.norm() * 57.3 < 0.01) &&
           (trans_delta.norm() * 100 < 0.015);
}

void UpdateCovarianceOnConverge(
    map::VoxelMapManager* map,
    const Eigen::Matrix<double, frontend::kDimState, frontend::kDimState>& G,
    const Eigen::Matrix<double, frontend::kDimState, frontend::kDimState>&
        I_STATE) {
    auto& state = map->state_;
    state.cov.block<frontend::kDimState, frontend::kDimState>(0, 0) =
        (I_STATE.block<frontend::kDimState, frontend::kDimState>(0, 0) -
         G.block<frontend::kDimState, frontend::kDimState>(0, 0)) *
        state.cov.block<frontend::kDimState, frontend::kDimState>(0, 0);
    map->position_last_ = state.pos_end;
    map->geo_quat_ = Eigen::Quaterniond(state.rot_end);
}

}  // namespace

void EsikfStateFilter::Update(map::VoxelMapManager* map,
                              frontend::StatesGroup* state_propagated) {
    Update(map, state_propagated, EsikfUpdateOptions{});
}

void EsikfStateFilter::Update(map::VoxelMapManager* map,
                              frontend::StatesGroup* state_propagated,
                              const EsikfUpdateOptions& options) {
    if (map == nullptr || state_propagated == nullptr) {
        return;
    }

    BuildBodyCovAndCrossMat(map);

    int rematch_count = 0;
    Eigen::Matrix<double, frontend::kDimState, frontend::kDimState> G, H_T_H,
        I_STATE;
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();

    Eigen::Matrix<double, frontend::kDimState, frontend::kDimState> cov_inv;
    bool cov_inv_ready = false;

    bool ekf_converged = false;
    bool stop_filter = false;
    auto& frame = map->frame();
    const auto& config = map->config_setting_;
    for (int iteration = 0; iteration < config.max_iterations_; ++iteration) {
        FillPvListWorldVars(map);
        frame.ptpl_list.clear();
        map->BuildResidualListOMP(frame.pv_list, frame.ptpl_list);

        BuildMeasurementJacobian(map, state_propagated);
        if (frame.effect_feat_num == 0) {
            continue;
        }

        stop_filter = false;
        ekf_converged = false;
        Eigen::VectorXd HTz = frame.Hsub_T_R_inv * frame.meas_vec;
        H_T_H.block<6, 6>(0, 0) = frame.Hsub_T_R_inv * frame.Hsub;

        // FIM → degeneracy → ConfidenceGating → inflate / skip / reweight.
        if (iteration == 0 &&
            (options.enable_degeneracy || options.enable_gating)) {
            if (ApplyDegeneracyAndGating(map, options, H_T_H, HTz)) {
                break;
            }
        }

        ekf_converged =
            SolveAndApplyEkfStep(map, state_propagated, G, H_T_H, HTz, cov_inv,
                                 cov_inv_ready);

        if (ekf_converged ||
            (rematch_count == 0 &&
             iteration == (config.max_iterations_ - 2))) {
            ++rematch_count;
        }

        if (!stop_filter &&
            (rematch_count >= 2 ||
             iteration == config.max_iterations_ - 1)) {
            UpdateCovarianceOnConverge(map, G, I_STATE);
            stop_filter = true;
        }
        if (stop_filter) {
            break;
        }
    }
}

}  // namespace backend

namespace map {

void VoxelMapManager::StateEstimation(StatesGroup& state_propagated) {
    backend::EsikfStateFilter::Update(this, &state_propagated);
}

}  // namespace map
}  // namespace argus

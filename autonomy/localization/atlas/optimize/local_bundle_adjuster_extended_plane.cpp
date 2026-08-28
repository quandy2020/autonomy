#include "autonomy/localization/atlas/optimize/local_bundle_adjuster_extended_plane.hpp"

#include "autonomy/localization/atlas/camera/perspective.hpp"
#include "autonomy/localization/atlas/data/keyframe.hpp"
#include "autonomy/localization/atlas/data/landmark.hpp"
#include "autonomy/localization/atlas/data/landmark_plane.hpp"
#include "autonomy/localization/atlas/data/map_database.hpp"
#include "autonomy/localization/atlas/optimize/g2o/extended/point2plane_distance_edge_wrapper.hpp"
#include "autonomy/localization/atlas/optimize/internal/landmark_vertex_container.hpp"
#include "autonomy/localization/atlas/optimize/internal/se3/reproj_edge_wrapper.hpp"
#include "autonomy/localization/atlas/optimize/internal/se3/shot_vertex_container.hpp"
#include "autonomy/localization/atlas/optimize/terminate_action.hpp"
#include "autonomy/localization/atlas/util/converter.hpp"

#include <unordered_map>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

namespace autonomy::localization::atlas::optimize {

local_bundle_adjuster_extended_plane::local_bundle_adjuster_extended_plane(const YAML::Node& yaml_node,
                                                                           const unsigned int num_first_iter,
                                                                           const unsigned int num_second_iter)
    : num_first_iter_(num_first_iter),
      num_second_iter_(num_second_iter),
      use_additional_keyframes_for_monocular_(yaml_node["use_additional_keyframes_for_monocular"].as<bool>(false)) {}

void local_bundle_adjuster_extended_plane::optimize(data::map_database* map_db,
                                                    const std::shared_ptr<data::keyframe>& curr_keyfrm,
                                                    bool* const force_stop_flag) const {
    std::unordered_map<unsigned int, std::shared_ptr<data::keyframe>> local_keyfrms;
    bool has_scale = false;

    local_keyfrms[curr_keyfrm->id_] = curr_keyfrm;
    for (const auto& local_keyfrm : curr_keyfrm->graph_node_->get_covisibilities()) {
        if (!local_keyfrm || local_keyfrm->will_be_erased() || local_keyfrm->graph_node_->is_spanning_root()) {
            continue;
        }
        if (local_keyfrm->id_ < map_db->get_fixed_keyframe_id_threshold()) {
            continue;
        }
        local_keyfrms[local_keyfrm->id_] = local_keyfrm;
        if (local_keyfrm->camera_->setup_type_ != camera::setup_type_t::Monocular) {
            has_scale = true;
        }
    }

    std::unordered_map<unsigned int, std::shared_ptr<data::landmark>> local_lms;
    std::unordered_map<unsigned int, Vec4_t> local_planes_function;

    for (const auto& id_keyfrm_pair : local_keyfrms) {
        for (const auto& local_lm : id_keyfrm_pair.second->get_landmarks()) {
            if (!local_lm || local_lm->will_be_erased() || local_lms.count(local_lm->id_)) {
                continue;
            }
            local_lms[local_lm->id_] = local_lm;

            const auto plane = local_lm->get_owning_plane();
            if (!plane || !plane->is_valid() || plane->need_refinement()) {
                continue;
            }
            if (local_planes_function.count(plane->id_)) {
                continue;
            }
            double a = 0, b = 0, c = 0, d = 0;
            plane->get_equation(a, b, c, d);
            local_planes_function[plane->id_] = Vec4_t(a, b, c, d);
        }
    }

    std::unordered_map<unsigned int, std::shared_ptr<data::keyframe>> fixed_keyfrms;
    for (const auto& local_lm : local_lms) {
        for (const auto& obs : local_lm.second->get_observations()) {
            const auto fixed_keyfrm = obs.first.lock();
            if (!fixed_keyfrm || fixed_keyfrm->will_be_erased()) {
                continue;
            }
            if (local_keyfrms.count(fixed_keyfrm->id_)) {
                continue;
            }
            if (fixed_keyfrms.count(fixed_keyfrm->id_)) {
                continue;
            }
            fixed_keyfrms[fixed_keyfrm->id_] = fixed_keyfrm;
        }
    }

    if (use_additional_keyframes_for_monocular_) {
        const auto additional_keyfrms_size = 2 - fixed_keyfrms.size();
        if (!has_scale && fixed_keyfrms.size() < 2 && local_keyfrms.size() > additional_keyfrms_size) {
            for (unsigned int i = 0; i < additional_keyfrms_size; ++i) {
                auto itr = local_keyfrms.begin();
                fixed_keyfrms[itr->first] = itr->second;
                local_keyfrms.erase(itr);
            }
        }
    }

    auto linear_solver =
        autonomy::localization::atlas::make_unique<::g2o::LinearSolverEigen<::g2o::BlockSolverX::PoseMatrixType>>();
    auto block_solver = autonomy::localization::atlas::make_unique<::g2o::BlockSolverX>(std::move(linear_solver));
    auto* algorithm = new ::g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    ::g2o::SparseOptimizer optimizer;
    auto* terminate_action_ptr = new terminate_action;
    terminate_action_ptr->setGainThreshold(1e-3);
    optimizer.addPostIterationAction(terminate_action_ptr);
    optimizer.setAlgorithm(algorithm);
    if (force_stop_flag) {
        optimizer.setForceStopFlag(force_stop_flag);
    }

    auto vtx_id_offset = std::make_shared<unsigned int>(0);
    internal::se3::shot_vertex_container keyfrm_vtx_container(vtx_id_offset,
                                                              local_keyfrms.size() + fixed_keyfrms.size());

    for (const auto& pair : local_keyfrms) {
        optimizer.addVertex(keyfrm_vtx_container.create_vertex(pair.second, false));
    }
    for (const auto& pair : fixed_keyfrms) {
        optimizer.addVertex(keyfrm_vtx_container.create_vertex(pair.second, true));
    }

    constexpr float chi_sq_2D = 5.99146f;
    const float sqrt_chi_sq_2D = std::sqrt(chi_sq_2D);
    constexpr float chi_sq_3D = 7.81473f;
    const float sqrt_chi_sq_3D = std::sqrt(chi_sq_3D);

    internal::landmark_vertex_container lm_vtx_container(vtx_id_offset, local_lms.size());
    using point_reproj_edge_wrapper = internal::se3::reproj_edge_wrapper<data::keyframe>;
    std::vector<point_reproj_edge_wrapper> reproj_edge_wraps;
    reproj_edge_wraps.reserve(local_lms.size() * 8);

    for (const auto& id_local_lm_pair : local_lms) {
        const auto& local_lm = id_local_lm_pair.second;
        if (local_lm->get_observations().empty()) {
            continue;
        }
        auto* lm_vtx = lm_vtx_container.create_vertex(local_lm, false);
        optimizer.addVertex(lm_vtx);

        for (const auto& obs : local_lm->get_observations()) {
            const auto keyfrm = obs.first.lock();
            const auto idx = obs.second;
            if (!keyfrm || keyfrm->will_be_erased() || !keyfrm_vtx_container.contain(keyfrm)) {
                continue;
            }
            const auto& undist_keypt = keyfrm->frm_obs_.undist_keypts_.at(idx);
            const float x_right =
                keyfrm->frm_obs_.stereo_x_right_.empty() ? -1.f : keyfrm->frm_obs_.stereo_x_right_.at(idx);
            const float inv_sigma_sq = keyfrm->orb_params_->inv_level_sigma_sq_.at(undist_keypt.octave);
            const float sqrt_chi_sq = (keyfrm->camera_->setup_type_ == camera::setup_type_t::Monocular)
                                          ? sqrt_chi_sq_2D
                                          : sqrt_chi_sq_3D;
            reproj_edge_wraps.emplace_back(keyfrm, keyfrm_vtx_container.get_vertex(keyfrm), local_lm, lm_vtx, idx,
                                           undist_keypt.pt.x, undist_keypt.pt.y, x_right, inv_sigma_sq, sqrt_chi_sq);
            optimizer.addEdge(reproj_edge_wraps.back().edge_);
        }
    }

    std::vector<plp_g2o::extended::point2plane_edge_wrapper> point2plane_edge_wraps;
    point2plane_edge_wraps.reserve(local_lms.size());
    for (const auto& id_local_lm_pair : local_lms) {
        const auto& local_lm = id_local_lm_pair.second;
        const auto plane = local_lm->get_owning_plane();
        if (!plane || !plane->is_valid() || !local_planes_function.count(plane->id_)) {
            continue;
        }
        auto* lm_vtx = lm_vtx_container.get_vertex(local_lm);
        point2plane_edge_wraps.emplace_back(local_lm, lm_vtx, local_planes_function.at(plane->id_));
        optimizer.addEdge(point2plane_edge_wraps.back().edge_);
    }

    if (force_stop_flag && *force_stop_flag) {
        delete terminate_action_ptr;
        return;
    }

    optimizer.initializeOptimization();
    optimizer.optimize(static_cast<int>(num_first_iter_));

    bool run_robust_ba = !(force_stop_flag && *force_stop_flag);
    if (run_robust_ba) {
        for (auto& reproj_edge_wrap : reproj_edge_wraps) {
            if (reproj_edge_wrap.lm_->will_be_erased()) {
                continue;
            }
            const bool outlier = reproj_edge_wrap.is_monocular_
                                     ? (chi_sq_2D < reproj_edge_wrap.edge_->chi2() || !reproj_edge_wrap.depth_is_positive())
                                     : (chi_sq_3D < reproj_edge_wrap.edge_->chi2() || !reproj_edge_wrap.depth_is_positive());
            if (outlier) {
                reproj_edge_wrap.set_as_outlier();
            }
            reproj_edge_wrap.edge_->setRobustKernel(nullptr);
        }
        for (auto& plane_edge_wrap : point2plane_edge_wraps) {
            if (!plane_edge_wrap.lm_ || plane_edge_wrap.lm_->will_be_erased()) {
                continue;
            }
            plane_edge_wrap.edge_->setRobustKernel(nullptr);
        }
        optimizer.initializeOptimization();
        optimizer.optimize(static_cast<int>(num_second_iter_));
    }

    delete terminate_action_ptr;

    std::vector<std::pair<std::shared_ptr<data::keyframe>, std::shared_ptr<data::landmark>>> outlier_observations;
    for (auto& reproj_edge_wrap : reproj_edge_wraps) {
        if (reproj_edge_wrap.lm_->will_be_erased()) {
            continue;
        }
        const bool outlier = reproj_edge_wrap.is_monocular_
                                 ? (chi_sq_2D < reproj_edge_wrap.edge_->chi2() || !reproj_edge_wrap.depth_is_positive())
                                 : (chi_sq_3D < reproj_edge_wrap.edge_->chi2() || !reproj_edge_wrap.depth_is_positive());
        if (outlier) {
            outlier_observations.emplace_back(reproj_edge_wrap.shot_, reproj_edge_wrap.lm_);
        }
    }

    {
        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

        for (const auto& outlier_obs : outlier_observations) {
            outlier_obs.first->erase_landmark(outlier_obs.second);
            outlier_obs.second->erase_observation(map_db, outlier_obs.first);
            if (!outlier_obs.second->will_be_erased()) {
                outlier_obs.second->compute_descriptor();
                outlier_obs.second->update_mean_normal_and_obs_scale_variance();
            }
        }

        for (const auto& pair : local_keyfrms) {
            auto* keyfrm_vtx = keyfrm_vtx_container.get_vertex(pair.second);
            pair.second->set_pose_cw(util::converter::to_eigen_mat(keyfrm_vtx->estimate()));
        }
        for (const auto& pair : local_lms) {
            if (pair.second->will_be_erased()) {
                continue;
            }
            pair.second->set_pos_in_world(lm_vtx_container.get_vertex(pair.second)->estimate());
            pair.second->update_mean_normal_and_obs_scale_variance();
        }
    }
}

}  // namespace autonomy::localization::atlas::optimize

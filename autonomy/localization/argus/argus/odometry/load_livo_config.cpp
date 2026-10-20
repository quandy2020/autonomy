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

#include "argus/odometry/load_livo_config.hpp"

#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "argus/common/yaml_parameter.hpp"

namespace argus {
namespace odometry {
namespace {

namespace fs = std::filesystem;

std::string DirOf(const std::string& path) {
    return fs::path(path).parent_path().string();
}

std::string JoinPath(const std::string& dir, const std::string& rel) {
    if (rel.empty()) {
        return {};
    }
    const fs::path p(rel);
    if (p.is_absolute()) {
        return p.string();
    }
    return (fs::path(dir) / p).string();
}

frontend::SlamMode SlamModeFromModalities(bool lidar, bool imu, bool cam) {
    if (lidar && imu && cam) {
        return frontend::kLivo;
    }
    if (lidar && imu) {
        return frontend::kOnlyLio;
    }
    if (lidar) {
        return frontend::kOnlyLo;
    }
    if (cam) {
        return frontend::kOnlyVio;  // IMU optional; image-timed VIO packages.
    }
    (void)imu;
    return frontend::kOnlyLio;
}

void LoadDegeneracyYaml(const std::string& path,
                        perception::DegeneracyConfig* out) {
    if (out == nullptr || path.empty()) {
        return;
    }
    common::YamlParameter y(path);
    if (!y.IsOpen()) {
        return;
    }
    out->eigenvalue_threshold =
        y.Get<double>("eigenvalue_threshold", out->eigenvalue_threshold);
    out->condition_number_threshold = y.Get<double>(
        "condition_number_threshold", out->condition_number_threshold);
    // Prefer enable_anisotropic_inflation; fall back to legacy key.
    out->enable_anisotropic_inflation =
        y.Get<int>("enable_anisotropic_inflation",
                   y.Get<int>("anisotropic_inflation",
                              out->enable_anisotropic_inflation ? 1 : 0)) != 0;
    out->inflation_scale =
        y.Get<double>("inflation_scale", out->inflation_scale);
    out->position_block_only =
        y.Get<int>("position_block_only", out->position_block_only ? 1 : 0) !=
        0;
}

void LoadGatingYaml(const std::string& path,
                    fusion::ConfidenceGatingConfig* out) {
    if (out == nullptr || path.empty()) {
        return;
    }
    common::YamlParameter y(path);
    if (!y.IsOpen()) {
        return;
    }
    out->min_lidar_weight =
        y.Get<double>("min_lidar_weight", out->min_lidar_weight);
    out->min_visual_weight =
        y.Get<double>("min_visual_weight", out->min_visual_weight);
    // Prefer min_inv_condition; fall back to legacy skip_on_degeneracy_ratio
    // where skip if inv_cond < (1 - ratio).
    if (y.Has("min_inv_condition")) {
        out->min_inv_condition =
            y.Get<double>("min_inv_condition", out->min_inv_condition);
    } else if (y.Has("skip_on_degeneracy_ratio")) {
        const double skip_ratio =
            y.Get<double>("skip_on_degeneracy_ratio", 0.7);
        out->min_inv_condition = 1.0 - skip_ratio;
    }
    out->max_dynamic_ratio_lidar =
        y.Get<double>("max_dynamic_ratio_lidar", out->max_dynamic_ratio_lidar);
    out->max_dynamic_ratio_visual = y.Get<double>(
        "max_dynamic_ratio_visual", out->max_dynamic_ratio_visual);
}

void LoadDynamicFilterYaml(const std::string& path,
                           frontend::DynamicFilterFactoryConfig* out) {
    if (out == nullptr || path.empty()) {
        return;
    }
    common::YamlParameter y(path);
    if (!y.IsOpen()) {
        return;
    }
    out->lidar =
        frontend::ParseDynamicFilterType(y.Get<std::string>("lidar", "none"));
    out->visual =
        frontend::ParseDynamicFilterType(y.Get<std::string>("visual", "none"));
    out->semantic_model =
        y.Get<std::string>("semantic_model", out->semantic_model);
    out->mask_threshold = y.Get<double>("mask_threshold", out->mask_threshold);
    out->semantic_backend =
        y.Get<std::string>("semantic/backend", out->semantic_backend);
    out->semantic_mask_path =
        y.Get<std::string>("semantic/mask_path", out->semantic_mask_path);
    out->semantic_input_width =
        y.Get<int>("semantic/input_width", out->semantic_input_width);
    out->semantic_input_height =
        y.Get<int>("semantic/input_height", out->semantic_input_height);
    {
        const auto ids_d = y.GetDoubleVector("semantic/person_class_ids");
        if (!ids_d.empty()) {
            out->semantic_dynamic_class_ids.clear();
            out->semantic_dynamic_class_ids.reserve(ids_d.size());
            for (double v : ids_d) {
                out->semantic_dynamic_class_ids.push_back(static_cast<int>(v));
            }
        }
    }
    out->spatiotemporal_range_diff_m =
        y.Get<double>("spatiotemporal/range_diff_m", out->spatiotemporal_range_diff_m);
    out->spatiotemporal_azimuth_res_deg = y.Get<double>("spatiotemporal/azimuth_res_deg",
                                            out->spatiotemporal_azimuth_res_deg);
    out->spatiotemporal_elevation_res_deg = y.Get<double>(
        "spatiotemporal/elevation_res_deg", out->spatiotemporal_elevation_res_deg);
    out->spatiotemporal_min_range_m =
        y.Get<double>("spatiotemporal/min_range_m", out->spatiotemporal_min_range_m);
    out->spatiotemporal_max_range_m =
        y.Get<double>("spatiotemporal/max_range_m", out->spatiotemporal_max_range_m);
}

void LoadMapCleaningYaml(const std::string& path,
                         map::MapSelfCleaningConfig* out) {
    if (out == nullptr || path.empty()) {
        return;
    }
    common::YamlParameter y(path);
    if (!y.IsOpen()) {
        return;
    }
    out->enable = y.Get<int>("enable", out->enable ? 1 : 0) != 0;
    out->interval_sec = y.Get<double>("interval_sec", out->interval_sec);
    out->confidence_threshold = static_cast<float>(
        y.Get<double>("confidence_threshold", out->confidence_threshold));
    out->degeneracy_penalty = static_cast<float>(
        y.Get<double>("degeneracy_penalty", out->degeneracy_penalty));
    out->min_observation_count_to_keep = static_cast<uint32_t>(
        y.Get<int>("min_observation_count_to_keep",
                   static_cast<int>(out->min_observation_count_to_keep)));
}

void LoadLoopClosureYaml(const std::string& path,
                         loop_closure::LoopClosureConfig* out,
                         std::string* error) {
    if (out == nullptr || path.empty()) {
        return;
    }
    common::YamlParameter y(path);
    if (!y.IsOpen()) {
        return;
    }
    out->enable = y.Get<int>("enable", out->enable ? 1 : 0) != 0;
    const int async = y.Get<int>("async", 1);
    if (async == 0) {
        if (error != nullptr) {
            *error = "loop_closure.async must be true (refuse sync Optimize)";
        }
        return;
    }
    out->async = true;
    out->current_session_id =
        y.Get<std::string>("current_session_id", out->current_session_id);
    // Prefer new keys; fall back to legacy names.
    out->enable_pose_inject =
        y.Get<int>("enable_pose_inject",
                   y.Get<int>("inject_pose_correction",
                              out->enable_pose_inject ? 1 : 0)) != 0;
    out->consensus.min_confidence =
        y.Get<double>("min_confidence", out->consensus.min_confidence);
    out->consensus.min_id_gap =
        y.Get<int>("consensus/min_id_gap",
                   y.Get<int>("min_id_gap", out->consensus.min_id_gap));
    out->consensus.min_path_separation_m = y.Get<double>(
        "consensus/min_path_separation_m",
        y.Get<double>("min_path_separation_m",
                      out->consensus.min_path_separation_m));
    out->scan_context.num_rings = y.Get<int>(
        "scan_context/num_rings",
        y.Get<int>("scan_context/ring_num", out->scan_context.num_rings));
    out->scan_context.num_sectors = y.Get<int>(
        "scan_context/num_sectors",
        y.Get<int>("scan_context/sector_num", out->scan_context.num_sectors));
    out->scan_context.max_radius_m = y.Get<double>(
        "scan_context/max_radius_m", out->scan_context.max_radius_m);
    out->scan_context.max_candidates = y.Get<int>(
        "scan_context/max_candidates",
        y.Get<int>("scan_context/top_k", out->scan_context.max_candidates));
    out->scan_context.exclude_recent_count = y.Get<int>(
        "scan_context/exclude_recent_count",
        out->scan_context.exclude_recent_count);
    out->scan_context.min_score = y.Get<double>(
        "scan_context/min_score", out->scan_context.min_score);
    out->scan_context.enable_lateral_search =
        y.Get<int>("scan_context/enable_lateral_search",
                   out->scan_context.enable_lateral_search ? 1 : 0) != 0;
    out->scan_context.lateral_max_shift = y.Get<int>(
        "scan_context/lateral_max_shift", out->scan_context.lateral_max_shift);
    out->scan_context.lateral_step_m = y.Get<double>(
        "scan_context/lateral_step_m", out->scan_context.lateral_step_m);
    out->consensus.geometric_weight = y.Get<double>(
        "consensus/geometric_weight", out->consensus.geometric_weight);
    out->consensus.visual_weight =
        y.Get<double>("consensus/visual_weight", out->consensus.visual_weight);
    out->consensus.topology_weight = y.Get<double>(
        "consensus/topology_weight", out->consensus.topology_weight);
    out->consensus.pose_consistency_weight =
        y.Get<double>("consensus/pose_consistency_weight",
                      out->consensus.pose_consistency_weight);
    out->consensus.max_relative_yaw_rad = y.Get<double>(
        "consensus/max_relative_yaw_rad",
        out->consensus.max_relative_yaw_rad);
    out->consensus.max_relative_trans_m = y.Get<double>(
        "consensus/max_relative_trans_m",
        out->consensus.max_relative_trans_m);
    out->cross_session.enable =
        y.Get<int>("cross_session/enable", out->cross_session.enable ? 1 : 0) !=
        0;
    out->cross_session.min_confidence = y.Get<double>(
        "cross_session/min_confidence", out->cross_session.min_confidence);
    out->cross_session.loop_information_scale =
        y.Get<double>("cross_session/loop_information_scale",
                      out->cross_session.loop_information_scale);
    out->cross_session.freeze_prior_nodes =
        y.Get<int>("cross_session/freeze_prior_nodes",
                   out->cross_session.freeze_prior_nodes ? 1 : 0) != 0;
    out->fpr.enable =
        y.Get<int>("fpr/enable", out->fpr.enable ? 1 : 0) != 0;
    out->fpr.max_loop_residual = y.Get<double>(
        "fpr/max_loop_residual", out->fpr.max_loop_residual);
    out->fpr.max_cost_rms =
        y.Get<double>("fpr/max_cost_rms", out->fpr.max_cost_rms);
    out->geometric_verify.enable =
        y.Get<int>("geometric_verify/enable",
                   out->geometric_verify.enable ? 1 : 0) != 0;
    out->geometric_verify.max_iterations = y.Get<int>(
        "geometric_verify/max_iterations",
        out->geometric_verify.max_iterations);
    out->geometric_verify.max_fitness_m = y.Get<double>(
        "geometric_verify/max_fitness_m",
        out->geometric_verify.max_fitness_m);
    out->geometric_verify.max_translation_m = y.Get<double>(
        "geometric_verify/max_translation_m",
        out->geometric_verify.max_translation_m);
    out->geometric_verify.max_yaw_rad = y.Get<double>(
        "geometric_verify/max_yaw_rad", out->geometric_verify.max_yaw_rad);
    out->geometric_verify.subsample_stride = y.Get<int>(
        "geometric_verify/subsample_stride",
        out->geometric_verify.subsample_stride);
    out->geometric_verify.min_correspondences = y.Get<int>(
        "geometric_verify/min_correspondences",
        out->geometric_verify.min_correspondences);
    out->geometric_verify.max_corr_dist_m = y.Get<double>(
        "geometric_verify/max_corr_dist_m",
        out->geometric_verify.max_corr_dist_m);
    out->geometric_verify.min_inlier_ratio = y.Get<double>(
        "geometric_verify/min_inlier_ratio",
        out->geometric_verify.min_inlier_ratio);
    out->geometric_verify.seed_xy_steps = y.Get<int>(
        "geometric_verify/seed_xy_steps",
        out->geometric_verify.seed_xy_steps);
    out->geometric_verify.seed_xy_step_m = y.Get<double>(
        "geometric_verify/seed_xy_step_m",
        out->geometric_verify.seed_xy_step_m);
    out->geometric_verify.seed_yaw_steps = y.Get<int>(
        "geometric_verify/seed_yaw_steps",
        out->geometric_verify.seed_yaw_steps);
    out->geometric_verify.seed_yaw_step_rad = y.Get<double>(
        "geometric_verify/seed_yaw_step_rad",
        out->geometric_verify.seed_yaw_step_rad);
    out->wait_second_loop =
        y.Get<int>("wait_second_loop", out->wait_second_loop ? 1 : 0) != 0;
    out->max_loop_translation_m = y.Get<double>(
        "max_loop_translation_m", out->max_loop_translation_m);
    out->max_inject_translation_m = y.Get<double>(
        "max_inject_translation_m", out->max_inject_translation_m);
    out->loop_information_scale = y.Get<double>(
        "loop_information_scale", out->loop_information_scale);
    out->inject_clamp_only =
        y.Get<int>("inject_clamp_only", out->inject_clamp_only ? 1 : 0) != 0;
    out->temporal.enable =
        y.Get<int>("temporal/enable", out->temporal.enable ? 1 : 0) != 0;
    out->temporal.min_confirmations = y.Get<int>(
        "temporal/min_confirmations", out->temporal.min_confirmations);
    out->temporal.max_match_id_delta = y.Get<int>(
        "temporal/max_match_id_delta", out->temporal.max_match_id_delta);
    out->temporal.max_query_span = y.Get<int>(
        "temporal/max_query_span", out->temporal.max_query_span);
    out->revisit_signature.enable =
        y.Get<int>("revisit_signature/enable",
                   out->revisit_signature.enable ? 1 : 0) != 0;
    out->revisit_signature.min_odom_gap_m = y.Get<double>(
        "revisit_signature/min_odom_gap_m",
        out->revisit_signature.min_odom_gap_m);
    out->revisit_signature.max_icp_trans_m = y.Get<double>(
        "revisit_signature/max_icp_trans_m",
        out->revisit_signature.max_icp_trans_m);
    out->revisit_signature.min_gap_minus_icp_m = y.Get<double>(
        "revisit_signature/min_gap_minus_icp_m",
        out->revisit_signature.min_gap_minus_icp_m);
    out->revisit_signature.max_fitness_m = y.Get<double>(
        "revisit_signature/max_fitness_m",
        out->revisit_signature.max_fitness_m);
    out->lta.enable = y.Get<int>("lta/enable", out->lta.enable ? 1 : 0) != 0;
    out->lta.points_per_frame = static_cast<std::size_t>(y.Get<int>(
        "lta/points_per_frame", static_cast<int>(out->lta.points_per_frame)));
    out->lta.max_queued_clouds = static_cast<std::size_t>(y.Get<int>(
        "lta/max_queued_clouds", static_cast<int>(out->lta.max_queued_clouds)));
    out->lta.downsample_resolution_m = y.Get<double>(
        "lta/downsample_resolution_m", out->lta.downsample_resolution_m);
    out->lta.prior_neighbor_keyframes = y.Get<int>(
        "lta/prior_neighbor_keyframes", out->lta.prior_neighbor_keyframes);
    out->lta_keyframe_buffer =
        y.Get<int>("lta/keyframe_buffer", out->lta_keyframe_buffer);
    out->lta_reload_prior_on_cross_session =
        y.Get<int>("lta/reload_prior_on_cross_session",
                   out->lta_reload_prior_on_cross_session ? 1 : 0) != 0;
}

void ParseModalities(const std::string& yaml_path, LivoOdometryConfig* cfg) {
    bool has_lidar = false;
    bool has_imu = false;
    bool has_cam = false;
    cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
    cv::FileNode mods = fs["odometry"]["modalities"];
    if (mods.isSeq()) {
        for (auto it = mods.begin(); it != mods.end(); ++it) {
            std::string m;
            try {
                (*it) >> m;
            } catch (const cv::Exception&) {
                continue;
            }
            if (m == "lidar") {
                has_lidar = true;
            } else if (m == "imu") {
                has_imu = true;
            } else if (m == "camera" || m == "image" || m == "visual") {
                has_cam = true;
            }
        }
    } else {
        has_lidar = true;
        has_imu = true;
    }
    cfg->enable_lidar = has_lidar;
    cfg->enable_imu = has_imu;
    cfg->enable_image = has_cam;
    cfg->slam_mode = SlamModeFromModalities(has_lidar, has_imu, has_cam);
}

void LoadDynamicFilterSection(const common::YamlParameter& root,
                              const std::string& base_dir,
                              LivoOdometryConfig* cfg) {
    cfg->dynamic_filter.lidar = frontend::ParseDynamicFilterType(
        root.Get<std::string>("frontend/dynamic_filter/lidar", "none"));
    cfg->dynamic_filter.visual = frontend::ParseDynamicFilterType(
        root.Get<std::string>("frontend/dynamic_filter/visual", "none"));
    cfg->dynamic_filter.semantic_model =
        root.Get<std::string>("frontend/dynamic_filter/semantic_model",
                              cfg->dynamic_filter.semantic_model);
    cfg->dynamic_filter.mask_threshold =
        root.Get<double>("frontend/dynamic_filter/mask_threshold",
                         cfg->dynamic_filter.mask_threshold);
    const std::string rel =
        root.Get<std::string>("frontend/dynamic_filter/config", "");
    LoadDynamicFilterYaml(JoinPath(base_dir, rel), &cfg->dynamic_filter);
}

void LoadRobustnessSection(const common::YamlParameter& root,
                           const std::string& base_dir,
                           LivoOdometryConfig* cfg) {
    cfg->enable_degeneracy =
        root.Get<int>("robustness/degeneracy/enable", 1) != 0;
    LoadDegeneracyYaml(
        JoinPath(base_dir,
                 root.Get<std::string>("robustness/degeneracy/config", "")),
        &cfg->degeneracy);

    cfg->enable_confidence_gating =
        root.Get<int>("robustness/confidence_gating/enable", 0) != 0;
    const std::string gating_rel =
        root.Get<std::string>("robustness/confidence_gating/config", "");
    LoadGatingYaml(JoinPath(base_dir, gating_rel), &cfg->confidence_gating);
    common::YamlParameter gy(JoinPath(base_dir, gating_rel));
    if (gy.IsOpen()) {
        cfg->sensor_timeout_sec =
            gy.Get<double>("sensor_timeout_sec", cfg->sensor_timeout_sec);
    }

    cfg->enable_intensity_assist =
        root.Get<int>("robustness/intensity_aided/enable", 1) != 0;
    cfg->intensity_assist.enable = cfg->enable_intensity_assist;
}

std::string LoadLoopClosureSection(const common::YamlParameter& root,
                                   const std::string& base_dir,
                                   LivoOdometryConfig* cfg) {
    cfg->loop_closure.enable = root.Get<int>("loop_closure/enable", 0) != 0;
    std::string lc_err;
    LoadLoopClosureYaml(
        JoinPath(base_dir, root.Get<std::string>("loop_closure/config", "")),
        &cfg->loop_closure, &lc_err);
    if (!lc_err.empty()) {
        return lc_err;
    }
    cfg->loop_closure.enable = root.Get<int>("loop_closure/enable", 0) != 0;
    if (root.Get<int>("cross_session/enable", 0) != 0) {
        cfg->loop_closure.cross_session.enable = true;
    }
    return {};
}

void LoadMappingSection(const common::YamlParameter& root,
                        LivoOdometryConfig* cfg) {
    cfg->enable_mapping = root.Get<int>("mapping/enable", 1) != 0;
    cfg->mapping.session_id =
        root.Get<std::string>("mapping/session_id", cfg->mapping.session_id);
    cfg->mapping_output_dir =
        root.Get<std::string>("mapping/output_dir", cfg->mapping_output_dir);
    cfg->mapping.ikd_downsample_resolution =
        root.Get<double>("mapping/ikd_downsample_resolution",
                         cfg->mapping.ikd_downsample_resolution);
    cfg->mapping.prior_resolution = root.Get<double>(
        "mapping/prior_resolution", cfg->mapping.prior_resolution);
}

void LoadRelocalizationSection(const common::YamlParameter& root,
                               LivoOdometryConfig* cfg) {
    auto& r = cfg->relocalization;
    r.enable = root.Get<int>("relocalization/enable", 1) != 0;
    r.trigger =
        root.Get<std::string>("relocalization/trigger", r.trigger);
    // Prefer min_lost_streak; fall back to legacy min_fail_streak.
    r.min_lost_streak = root.Get<int>(
        "relocalization/min_lost_streak",
        root.Get<int>("relocalization/min_fail_streak", r.min_lost_streak));
    r.cooldown_sec =
        root.Get<double>("relocalization/cooldown_sec", r.cooldown_sec);
    r.seed_with_current_pose =
        root.Get<int>("relocalization/seed_with_current_pose",
                      r.seed_with_current_pose ? 1 : 0) != 0;
    r.prior_map_path =
        root.Get<std::string>("relocalization/prior_map_path", r.prior_map_path);
    r.prior_resolution = root.Get<double>("relocalization/prior_resolution",
                                          r.prior_resolution);
    // Prefer worker_count; fall back to legacy size.
    r.thread_pool.worker_count = root.Get<int>(
        "relocalization/thread_pool/worker_count",
        root.Get<int>("relocalization/thread_pool/size",
                      r.thread_pool.worker_count));
    r.thread_pool.max_queue_size =
        root.Get<int>("relocalization/thread_pool/max_queue_size",
                      r.thread_pool.max_queue_size);
    // Prefer enable_low_priority; fall back to legacy low_priority.
    r.thread_pool.enable_low_priority =
        root.Get<int>("relocalization/thread_pool/enable_low_priority",
                      root.Get<int>("relocalization/thread_pool/low_priority",
                                    r.thread_pool.enable_low_priority ? 1
                                                                      : 0)) !=
        0;

    // Prefer branch_and_bound/*; fall back to legacy bnb/*.
    const char* bnb_prefix =
        root.Has("relocalization/branch_and_bound/max_depth") ||
                root.Has("relocalization/branch_and_bound/search_space")
            ? "relocalization/branch_and_bound/"
            : "relocalization/bnb/";
    auto bnb_key = [bnb_prefix](const char* leaf) {
        return std::string(bnb_prefix) + leaf;
    };
    r.branch_and_bound.max_depth = root.Get<int>(
        bnb_key("max_depth"), r.branch_and_bound.max_depth);
    r.branch_and_bound.resolution = root.Get<double>(
        bnb_key("resolution"), r.branch_and_bound.resolution);
    r.branch_and_bound.z_resolution = root.Get<double>(
        bnb_key("z_resolution"), r.branch_and_bound.z_resolution);
    r.branch_and_bound.score_threshold = root.Get<double>(
        bnb_key("score_threshold"), r.branch_and_bound.score_threshold);
    r.branch_and_bound.search_radius_z_steps =
        root.Get<int>(bnb_key("search_radius_z_steps"),
                      r.branch_and_bound.search_radius_z_steps);
    r.branch_and_bound.roll_pitch_refine_rad =
        root.Get<double>(bnb_key("roll_pitch_refine_rad"),
                         r.branch_and_bound.roll_pitch_refine_rad);
    r.branch_and_bound.roll_pitch_steps = root.Get<int>(
        bnb_key("roll_pitch_steps"), r.branch_and_bound.roll_pitch_steps);
    r.branch_and_bound.se3_trans_refine_m = root.Get<double>(
        bnb_key("se3_trans_refine_m"), r.branch_and_bound.se3_trans_refine_m);
    r.branch_and_bound.se3_yaw_refine_rad = root.Get<double>(
        bnb_key("se3_yaw_refine_rad"), r.branch_and_bound.se3_yaw_refine_rad);
    r.branch_and_bound.se3_refine_passes = root.Get<int>(
        bnb_key("se3_refine_passes"), r.branch_and_bound.se3_refine_passes);
    const std::string space =
        root.Get<std::string>(bnb_key("search_space"), "se3");
    if (space == "se2") {
        r.branch_and_bound.search_space = relocalization::BnBSearchSpace::kSE2;
    } else if (space == "se3_yaw") {
        r.branch_and_bound.search_space = relocalization::BnBSearchSpace::kSE3Yaw;
    } else {
        r.branch_and_bound.search_space = relocalization::BnBSearchSpace::kSE3;
    }

    // Optional Scan Context++ place crop before BnB.
    auto& pr = r.place_recognition;
    pr.enable =
        root.Get<int>("relocalization/place_recognition/enable",
                      pr.enable ? 1 : 0) != 0;
    pr.top_k = root.Get<int>("relocalization/place_recognition/top_k", pr.top_k);
    pr.min_score = root.Get<double>("relocalization/place_recognition/min_score",
                                    pr.min_score);
    pr.crop_radius_steps = root.Get<int>(
        "relocalization/place_recognition/crop_radius_steps",
        pr.crop_radius_steps);
    pr.fallback_full_search =
        root.Get<int>("relocalization/place_recognition/fallback_full_search",
                      pr.fallback_full_search ? 1 : 0) != 0;
    pr.num_rings = root.Get<int>("relocalization/place_recognition/num_rings",
                                 pr.num_rings);
    pr.num_sectors = root.Get<int>(
        "relocalization/place_recognition/num_sectors", pr.num_sectors);
    pr.max_radius_m = root.Get<double>(
        "relocalization/place_recognition/max_radius_m", pr.max_radius_m);
    pr.keyframe_spacing_m = root.Get<double>(
        "relocalization/place_recognition/keyframe_spacing_m",
        pr.keyframe_spacing_m);
    pr.min_points_per_keyframe = root.Get<int>(
        "relocalization/place_recognition/min_points_per_keyframe",
        pr.min_points_per_keyframe);
    pr.ring_key_prefilter_k = root.Get<int>(
        "relocalization/place_recognition/ring_key_prefilter_k",
        pr.ring_key_prefilter_k);
    pr.enable_lateral_search =
        root.Get<int>("relocalization/place_recognition/enable_lateral_search",
                      pr.enable_lateral_search ? 1 : 0) != 0;
    pr.lateral_max_shift = root.Get<int>(
        "relocalization/place_recognition/lateral_max_shift",
        pr.lateral_max_shift);
    pr.lateral_step_m = root.Get<double>(
        "relocalization/place_recognition/lateral_step_m", pr.lateral_step_m);
}

void LoadVioSection(const common::YamlParameter& root,
                    const std::string& base_dir, LivoOdometryConfig* cfg) {
    cfg->vio.camera_calib_file = JoinPath(
        base_dir, root.Get<std::string>("camera/calib_file", ""));
    cfg->vio.grid_size = root.Get<int>("vio/grid_size", cfg->vio.grid_size);
    cfg->vio.grid_n_width =
        root.Get<int>("vio/grid_n_width", cfg->vio.grid_n_width);
    cfg->vio.grid_n_height =
        root.Get<int>("vio/grid_n_height", cfg->vio.grid_n_height);
    cfg->vio.patch_size = root.Get<int>("vio/patch_size", cfg->vio.patch_size);
    // Prefer new keys; fall back to legacy FAST-LIVO-style names.
    cfg->vio.patch_pyramid_level = root.Get<int>(
        "vio/patch_pyramid_level",
        root.Get<int>("vio/patch_pyrimid_level", cfg->vio.patch_pyramid_level));
    cfg->vio.max_iterations =
        root.Get<int>("vio/max_iterations", cfg->vio.max_iterations);
    cfg->vio.outlier_threshold =
        root.Get<double>("vio/outlier_threshold", cfg->vio.outlier_threshold);
    // Prefer image_point_cov; fall back to legacy img_point_cov.
    cfg->vio.image_point_cov = root.Get<double>(
        "vio/image_point_cov",
        root.Get<double>("vio/img_point_cov", cfg->vio.image_point_cov));
    cfg->vio.enable_normal =
        root.Get<int>("vio/enable_normal",
                      root.Get<int>("vio/normal_en",
                                    cfg->vio.enable_normal ? 1 : 0)) != 0;
    cfg->vio.enable_inverse_composition = root.Get<int>(
                                              "vio/enable_inverse_composition",
                                              root.Get<int>(
                                                  "vio/inverse_composition_en",
                                                  cfg->vio.enable_inverse_composition
                                                      ? 1
                                                      : 0)) != 0;
    cfg->vio.enable_raycast =
        root.Get<int>("vio/enable_raycast",
                      root.Get<int>("vio/raycast_en",
                                    cfg->vio.enable_raycast ? 1 : 0)) != 0;
    cfg->vio.enable_exposure_estimate = root.Get<int>(
                                            "vio/enable_exposure_estimate",
                                            root.Get<int>(
                                                "vio/exposure_estimate_en",
                                                cfg->vio.enable_exposure_estimate
                                                    ? 1
                                                    : 0)) != 0;
    cfg->vio.enable_colmap_output = root.Get<int>(
                                        "vio/enable_colmap_output",
                                        root.Get<int>("vio/colmap_output_en",
                                                      cfg->vio.enable_colmap_output
                                                          ? 1
                                                          : 0)) != 0;
    cfg->vio.log_dir = root.Get<std::string>("vio/log_dir", cfg->vio.log_dir);
}

void ReadCvDoubleSeq(cv::FileNode n, std::vector<double>* out,
                     std::size_t expect) {
    if (!n.isSeq() || static_cast<std::size_t>(n.size()) != expect) {
        return;
    }
    out->clear();
    out->reserve(expect);
    for (auto it = n.begin(); it != n.end(); ++it) {
        double v = 0.0;
        try {
            (*it) >> v;
        } catch (const cv::Exception&) {
            continue;
        }
        out->push_back(v);
    }
}

void FillMat3(const std::vector<double>& v, Eigen::Matrix3d* M) {
    if (M == nullptr || v.size() != 9) {
        return;
    }
    *M << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
}

void FillVec3(const std::vector<double>& v, Eigen::Vector3d* t) {
    if (t == nullptr || v.size() != 3) {
        return;
    }
    *t << v[0], v[1], v[2];
}

void LoadExtrinsics(const common::YamlParameter& root,
                    const std::string& yaml_path, const std::string& base_dir,
                    LivoOdometryConfig* cfg) {
    const std::string lc_rel =
        root.Get<std::string>("extrinsic/lidar_camera", "");
    if (!lc_rel.empty()) {
        cv::FileStorage lfs(JoinPath(base_dir, lc_rel), cv::FileStorage::READ);
        if (lfs.isOpened()) {
            ReadCvDoubleSeq(lfs["Rcl"], &cfg->vio.Rcl, 9);
            ReadCvDoubleSeq(lfs["Pcl"], &cfg->vio.Pcl, 3);
        }
    }
    const std::string il_rel = root.Get<std::string>("extrinsic/imu_lidar", "");
    if (!il_rel.empty()) {
        cv::FileStorage ifs(JoinPath(base_dir, il_rel), cv::FileStorage::READ);
        if (ifs.isOpened()) {
            std::vector<double> R;
            std::vector<double> T;
            ReadCvDoubleSeq(ifs["ext_R"], &R, 9);
            ReadCvDoubleSeq(ifs["ext_T"], &T, 3);
            if (R.size() != 9) {
                ReadCvDoubleSeq(ifs["R"], &R, 9);
            }
            if (T.size() != 3) {
                ReadCvDoubleSeq(ifs["T"], &T, 3);
            }
            FillMat3(R, &cfg->ext_R);
            FillVec3(T, &cfg->ext_T);
        }
    }
    cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
    if (cfg->vio.Rcl.size() != 9) {
        ReadCvDoubleSeq(fs["extrinsic"]["Rcl"], &cfg->vio.Rcl, 9);
    }
    if (cfg->vio.Pcl.size() != 3) {
        ReadCvDoubleSeq(fs["extrinsic"]["Pcl"], &cfg->vio.Pcl, 3);
    }
    if (cfg->vio.Rcl.size() != 9) {
        ReadCvDoubleSeq(fs["extrin_calib"]["Rcl"], &cfg->vio.Rcl, 9);
    }
    if (cfg->vio.Pcl.size() != 3) {
        ReadCvDoubleSeq(fs["extrin_calib"]["Pcl"], &cfg->vio.Pcl, 3);
    }
}

std::string ValidateLivoConfig(const LivoOdometryConfig& cfg) {
    if (cfg.enable_image) {
        if (cfg.vio.camera_calib_file.empty() ||
            !std::filesystem::exists(cfg.vio.camera_calib_file)) {
            return "camera modality enabled but camera/calib_file missing or "
                   "unreadable: " +
                   cfg.vio.camera_calib_file;
        }
    }
    if (!cfg.enable_lidar && cfg.enable_image) {
        return "modalities without lidar (ONLY_VIO) are not production-ready; "
               "require lidar+camera (LIVO) or lidar(+imu). Use LO/LIO/LIVO.";
    }
    if (cfg.dynamic_filter.lidar == frontend::DynamicFilterType::kSemantic ||
        cfg.dynamic_filter.visual == frontend::DynamicFilterType::kSemantic) {
        if (cfg.dynamic_filter.semantic_backend == "onnx" ||
            cfg.dynamic_filter.semantic_backend.empty()) {
            if (cfg.dynamic_filter.semantic_model.empty()) {
                return "dynamic_filter semantic backend=onnx requires "
                       "semantic_model path (or set semantic/backend: "
                       "mask_file)";
            }
        } else if (cfg.dynamic_filter.semantic_backend != "mask_file") {
            return "unknown semantic/backend \"" +
                   cfg.dynamic_filter.semantic_backend +
                   "\" (use onnx|mask_file)";
        }
    }
    const std::string& trig = cfg.relocalization.trigger;
    if (trig != "tracking_lost" && trig != "manual" &&
        trig != "cross_session_load") {
        return "relocalization.trigger must be tracking_lost|manual|"
               "cross_session_load (got \"" +
               trig + "\")";
    }
    if (cfg.loop_closure.enable && !cfg.loop_closure.async) {
        return "loop_closure.async must be true when enable=true";
    }
    return {};
}

}  // namespace

LoadLivoConfigResult LoadLivoOdometryConfig(const std::string& yaml_path) {
    LoadLivoConfigResult result;
    result.main_config_path = yaml_path;

    common::YamlParameter root(yaml_path);
    if (!root.IsOpen()) {
        result.error = "cannot open " + yaml_path;
        return result;
    }

    const std::string base_dir = DirOf(yaml_path);
    LivoOdometryConfig& cfg = result.config;

    ParseModalities(yaml_path, &cfg);
    cfg.voxel.max_iterations_ =
        root.Get<int>("backend/max_iterations", cfg.voxel.max_iterations_);
    cfg.voxel.max_voxel_size_ =
        root.Get<double>("map/lidar/resolution", cfg.voxel.max_voxel_size_);
    cfg.surface_voxel_size = cfg.voxel.max_voxel_size_;

    const int backend_threads = root.Get<int>("threading/backend_threads", 1);
    if (backend_threads != 1) {
        result.error = "threading.backend_threads must be 1 (got " +
                       std::to_string(backend_threads) + ")";
        return result;
    }

    LoadDynamicFilterSection(root, base_dir, &cfg);
    LoadRobustnessSection(root, base_dir, &cfg);

    cfg.map_cleaning.enable = root.Get<int>("map/self_cleaning/enable", 0) != 0;
    LoadMapCleaningYaml(
        JoinPath(base_dir, root.Get<std::string>("map/self_cleaning/config", "")),
        &cfg.map_cleaning);
    if (root.Has("map/self_cleaning/enable")) {
        cfg.map_cleaning.enable =
            root.Get<int>("map/self_cleaning/enable", 0) != 0;
    }

    result.error = LoadLoopClosureSection(root, base_dir, &cfg);
    if (!result.error.empty()) {
        return result;
    }

    LoadMappingSection(root, &cfg);
    cfg.synthetic_drift.enable =
        root.Get<int>("synthetic_drift/enable",
                      cfg.synthetic_drift.enable ? 1 : 0) != 0;
    cfg.synthetic_drift.yaw_rad_per_m = root.Get<double>(
        "synthetic_drift/yaw_rad_per_m", cfg.synthetic_drift.yaw_rad_per_m);
    cfg.synthetic_drift.tx_m_per_m = root.Get<double>(
        "synthetic_drift/tx_m_per_m", cfg.synthetic_drift.tx_m_per_m);
    cfg.synthetic_drift.step_m =
        root.Get<double>("synthetic_drift/step_m", cfg.synthetic_drift.step_m);
    LoadRelocalizationSection(root, &cfg);

    if (root.Get<int>("threading/use_global_thread_pool", 0) != 0) {
        result.error =
            "threading.use_global_thread_pool must be false "
            "(relocalization owns an isolated pool)";
        return result;
    }

    LoadVioSection(root, base_dir, &cfg);
    LoadExtrinsics(root, yaml_path, base_dir, &cfg);

    result.error = ValidateLivoConfig(cfg);
    if (!result.error.empty()) {
        return result;
    }

    result.ok = true;
    return result;
}

}  // namespace odometry
}  // namespace argus

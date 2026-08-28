/*
 * Copyright 2026 The Openbot Authors
 *
 * Structure-PLP-SLAM integration options (point-line-plane sparse SLAM).
 * Reference: Structure PLP-SLAM (DFKI), based on OpenVSLAM v0.3.9.
 */

#pragma once

#include <string>

#include <yaml-cpp/yaml.h>

namespace autonomy::localization::atlas::plp {

/** YAML section: PLP (mirrors Structure-PLP-SLAM planar_mapping_parameters.yaml). */
struct Options {
    bool enabled = false;
    bool use_line_tracking = true;
    bool use_plane_mapping = false;

    // Line (LSD/LBD) — PLPSLAM feature/line_extractor defaults
    float min_line_length_ratio = 0.125f;
    int min_line_pixels = 60;
    bool equalize_histogram = true;

    // Plane (segmentation-driven) — requires external mask per frame
    bool use_graph_cut_ransac = true;
    bool check_seg_3x3_window = true;
    int min_points_before_ransac = 12;
    int points_per_ransac = 12;
    int ransac_iterations = 50;
    float plane_distance_correction = 0.02f;
    float final_error_correction = 0.01f;
    float inliers_ratio_thr = 0.7f;
    float dot_product_threshold = 0.8f;
    float offset_delta_factor = 6.f;
    float gc_confidence = 0.99f;
    float spatial_coherence_weight = 0.6f;
    int gc_fps = -1;
    float gc_sphere_radius = 0.1f;
    int gc_adaptive_number = 6;
    float gc_minimum_inlier_ratio_for_sprt = 0.05f;
    bool set_verbose = false;
    /** If false, keep ORB landmark positions (safer when segmentation is noisy). */
    bool refine_landmark_positions = false;

    /** Load from Atlas config YAML (section "PLP"). */
    static Options FromYaml(const YAML::Node& root);
};

}  // namespace autonomy::localization::atlas::plp

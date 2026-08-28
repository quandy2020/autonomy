/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/localization/atlas/plp/plp_options.hpp"

#include "autonomy/localization/atlas/util/yaml.hpp"

namespace autonomy::localization::atlas::plp {

Options Options::FromYaml(const YAML::Node& root) {
    Options o;
    const auto node = util::yaml_optional_ref(root, "PLP");
    if (!node || node.IsNull()) {
        return o;
    }
    o.enabled = node["enabled"].as<bool>(o.enabled);
    o.use_line_tracking = node["use_line_tracking"].as<bool>(o.use_line_tracking);
    o.use_plane_mapping = node["use_plane_mapping"].as<bool>(o.use_plane_mapping);

    if (const auto line = node["Line"]) {
        o.min_line_length_ratio =
            line["min_line_length_ratio"].as<float>(o.min_line_length_ratio);
        o.min_line_pixels = line["min_line_pixels"].as<int>(o.min_line_pixels);
        o.equalize_histogram =
            line["equalize_histogram"].as<bool>(o.equalize_histogram);
    }
    if (const auto plane = node["Plane"]) {
        o.use_graph_cut_ransac =
            plane["use_graph_cut_ransac"].as<bool>(o.use_graph_cut_ransac);
        o.check_seg_3x3_window =
            plane["check_seg_3x3_window"].as<bool>(o.check_seg_3x3_window);
        o.min_points_before_ransac =
            plane["min_points_before_ransac"].as<int>(o.min_points_before_ransac);
        o.points_per_ransac =
            plane["points_per_ransac"].as<int>(o.points_per_ransac);
        o.ransac_iterations =
            plane["ransac_iterations"].as<int>(o.ransac_iterations);
        o.plane_distance_correction =
            plane["plane_distance_correction"].as<float>(o.plane_distance_correction);
        o.final_error_correction =
            plane["final_error_correction"].as<float>(o.final_error_correction);
        o.inliers_ratio_thr =
            plane["inliers_ratio_thr"].as<float>(o.inliers_ratio_thr);
        o.dot_product_threshold =
            plane["dot_product_threshold"].as<float>(o.dot_product_threshold);
        o.offset_delta_factor =
            plane["offset_delta_factor"].as<float>(o.offset_delta_factor);
        o.gc_confidence = plane["gc_confidence"].as<float>(o.gc_confidence);
        o.spatial_coherence_weight =
            plane["spatial_coherence_weight"].as<float>(o.spatial_coherence_weight);
        o.gc_fps = plane["gc_fps"].as<int>(o.gc_fps);
        o.gc_sphere_radius = plane["gc_sphere_radius"].as<float>(o.gc_sphere_radius);
        o.gc_adaptive_number = plane["gc_adaptive_number"].as<int>(o.gc_adaptive_number);
        o.gc_minimum_inlier_ratio_for_sprt =
            plane["gc_minimum_inlier_ratio_for_sprt"].as<float>(o.gc_minimum_inlier_ratio_for_sprt);
        o.set_verbose = plane["set_verbose"].as<bool>(o.set_verbose);
        o.refine_landmark_positions =
            plane["refine_landmark_positions"].as<bool>(o.refine_landmark_positions);
    }
    if (o.use_plane_mapping) {
        o.enabled = true;
    }
    if (o.use_line_tracking) {
        o.enabled = true;
    }
    return o;
}

}  // namespace autonomy::localization::atlas::plp

/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/core/exploration_options.hpp"

#include <cmath>

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"

namespace autonomy::perception::exploration {
namespace {

using ::autonomy::common::ConfigurationFileResolver;
using ::autonomy::common::LuaParameterDictionary;

void ApplyLua(LuaParameterDictionary* dict, proto::ExplorationOptions* opts) {
  if (dict->HasKey("map_frame")) {
    opts->set_map_frame(dict->GetString("map_frame"));
  }
  if (dict->HasKey("default_area_half_extent_m")) {
    opts->set_default_area_half_extent_m(
        dict->GetDouble("default_area_half_extent_m"));
  }
  if (dict->HasKey("completion_coverage_ratio")) {
    opts->set_completion_coverage_ratio(
        dict->GetDouble("completion_coverage_ratio"));
  }
  if (dict->HasKey("grid_cell_size_m")) {
    opts->set_grid_cell_size_m(dict->GetDouble("grid_cell_size_m"));
  }
  if (dict->HasKey("grid_cols")) {
    opts->set_grid_cols(dict->GetInt("grid_cols"));
  }
  if (dict->HasKey("grid_rows")) {
    opts->set_grid_rows(dict->GetInt("grid_rows"));
  }
  if (dict->HasKey("sensor_range_m")) {
    opts->set_sensor_range_m(dict->GetDouble("sensor_range_m"));
  }
  if (dict->HasKey("lookahead_distance_m")) {
    opts->set_lookahead_distance_m(dict->GetDouble("lookahead_distance_m"));
  }
  if (dict->HasKey("costmap_resolution_m")) {
    opts->set_costmap_resolution_m(dict->GetDouble("costmap_resolution_m"));
  }
  if (dict->HasKey("costmap_size_cells")) {
    opts->set_costmap_size_cells(dict->GetInt("costmap_size_cells"));
  }
  if (dict->HasKey("explorer_backend")) {
    opts->set_explorer_backend(dict->GetString("explorer_backend"));
  }
  if (dict->HasKey("far_connect_dist_m")) {
    opts->set_far_connect_dist_m(dict->GetDouble("far_connect_dist_m"));
  }
  if (dict->HasKey("far_contour_stride_cells")) {
    opts->set_far_contour_stride_cells(dict->GetInt("far_contour_stride_cells"));
  }
  if (dict->HasKey("far_attempt_unknown")) {
    opts->set_far_attempt_unknown(dict->GetBool("far_attempt_unknown"));
  }
  if (dict->HasKey("far3d_voxel_dim_m")) {
    opts->set_far3d_voxel_dim_m(dict->GetDouble("far3d_voxel_dim_m"));
  }
  if (dict->HasKey("far3d_contour_thresh")) {
    opts->set_far3d_contour_thresh(dict->GetInt("far3d_contour_thresh"));
  }
  if (dict->HasKey("far3d_contour_blur_size")) {
    opts->set_far3d_contour_blur_size(dict->GetInt("far3d_contour_blur_size"));
  }
  if (dict->HasKey("far3d_graph_pool_size")) {
    opts->set_far3d_graph_pool_size(dict->GetInt("far3d_graph_pool_size"));
  }
  if (dict->HasKey("far3d_edge_finalize_thred")) {
    opts->set_far3d_edge_finalize_thred(
        dict->GetInt("far3d_edge_finalize_thred"));
  }
  if (dict->HasKey("far3d_height_margin_m")) {
    opts->set_far3d_height_margin_m(dict->GetDouble("far3d_height_margin_m"));
  }
  if (dict->HasKey("far3d_position_filter_m")) {
    opts->set_far3d_position_filter_m(
        dict->GetDouble("far3d_position_filter_m"));
  }
  if (dict->HasKey("far3d_pillar_perimeter_m")) {
    opts->set_far3d_pillar_perimeter_m(
        dict->GetDouble("far3d_pillar_perimeter_m"));
  }
  if (dict->HasKey("far3d_dumper_thred")) {
    opts->set_far3d_dumper_thred(dict->GetInt("far3d_dumper_thred"));
  }
  if (dict->HasKey("viewpoint_num_z")) {
    opts->set_viewpoint_num_z(dict->GetInt("viewpoint_num_z"));
  }
  if (dict->HasKey("viewpoint_resolution_z_m")) {
    opts->set_viewpoint_resolution_z_m(dict->GetDouble("viewpoint_resolution_z_m"));
  }
  if (dict->HasKey("use_momentum")) {
    opts->set_use_momentum(dict->GetBool("use_momentum"));
  }
  if (dict->HasKey("momentum_dist_m")) {
    opts->set_momentum_dist_m(dict->GetDouble("momentum_dist_m"));
  }
  if (dict->HasKey("momentum_thred")) {
    opts->set_momentum_thred(dict->GetInt("momentum_thred"));
  }
  if (dict->HasKey("local_path_optimization_iters")) {
    opts->set_local_path_optimization_iters(
        dict->GetInt("local_path_optimization_iters"));
  }
  if (dict->HasKey("use_frontier_points")) {
    opts->set_use_frontier_points(dict->GetBool("use_frontier_points"));
  }
  if (dict->HasKey("use_terrain_height")) {
    opts->set_use_terrain_height(dict->GetBool("use_terrain_height"));
  }
  if (dict->HasKey("nogo_half_extent_m")) {
    opts->set_nogo_half_extent_m(dict->GetDouble("nogo_half_extent_m"));
  }
  if (dict->HasKey("point_cloud_voxel_m")) {
    opts->set_point_cloud_voxel_m(dict->GetDouble("point_cloud_voxel_m"));
  }
  if (dict->HasKey("point_cloud_min_points_per_voxel")) {
    opts->set_point_cloud_min_points_per_voxel(
        dict->GetInt("point_cloud_min_points_per_voxel"));
  }
  if (dict->HasKey("lidar_h_res_deg")) {
    opts->set_lidar_h_res_deg(dict->GetDouble("lidar_h_res_deg"));
  }
  if (dict->HasKey("lidar_v_res_deg")) {
    opts->set_lidar_v_res_deg(dict->GetDouble("lidar_v_res_deg"));
  }
  if (dict->HasKey("tsp_use_two_opt")) {
    opts->set_tsp_use_two_opt(dict->GetBool("tsp_use_two_opt"));
  }
  if (dict->HasKey("far_is_autoswitch")) {
    opts->set_far_is_autoswitch(dict->GetBool("far_is_autoswitch"));
  }
  if (dict->HasKey("far_viewpoint_extend")) {
    opts->set_far_viewpoint_extend(dict->GetBool("far_viewpoint_extend"));
  }
  if (dict->HasKey("far_local_planner_range_m")) {
    opts->set_far_local_planner_range_m(dict->GetDouble("far_local_planner_range_m"));
  }
  if (dict->HasKey("far_path_momentum_thred")) {
    opts->set_far_path_momentum_thred(dict->GetInt("far_path_momentum_thred"));
  }
  if (dict->HasKey("far_is_static_env")) {
    opts->set_far_is_static_env(dict->GetBool("far_is_static_env"));
  }
  if (dict->HasKey("far_converge_dist_m")) {
    opts->set_far_converge_dist_m(dict->GetDouble("far_converge_dist_m"));
  }
  if (dict->HasKey("far_terrain_grid_res_m")) {
    opts->set_far_terrain_grid_res_m(dict->GetDouble("far_terrain_grid_res_m"));
  }
  if (dict->HasKey("far_scan_voxel_m")) {
    opts->set_far_scan_voxel_m(dict->GetDouble("far_scan_voxel_m"));
  }
  if (dict->HasKey("boundary_ply_path")) {
    opts->set_boundary_ply_path(dict->GetString("boundary_ply_path"));
  }
  if (dict->HasKey("nogo_ply_path")) {
    opts->set_nogo_ply_path(dict->GetString("nogo_ply_path"));
  }
  if (dict->HasKey("tsp_use_ortools")) {
    opts->set_tsp_use_ortools(dict->GetBool("tsp_use_ortools"));
  }
  if (dict->HasKey("use_lidar_primary")) {
    opts->set_use_lidar_primary(dict->GetBool("use_lidar_primary"));
  }
  if (dict->HasKey("boundary_vgh_path")) {
    opts->set_boundary_vgh_path(dict->GetString("boundary_vgh_path"));
  }
  if (dict->HasKey("use_prior_map")) {
    opts->set_use_prior_map(dict->GetBool("use_prior_map"));
  }
  if (dict->HasKey("prior_map_topic")) {
    opts->set_prior_map_topic(dict->GetString("prior_map_topic"));
  }
  if (dict->HasKey("prior_map_lethal_threshold")) {
    opts->set_prior_map_lethal_threshold(dict->GetInt("prior_map_lethal_threshold"));
  }
  if (dict->HasKey("far2d_graph_pool_size")) {
    opts->set_far2d_graph_pool_size(dict->GetInt("far2d_graph_pool_size"));
  }
  if (dict->HasKey("far2d_dumper_thred")) {
    opts->set_far2d_dumper_thred(dict->GetInt("far2d_dumper_thred"));
  }
  if (dict->HasKey("far_goal_distance_weight")) {
    opts->set_far_goal_distance_weight(dict->GetDouble("far_goal_distance_weight"));
  }
  if (dict->HasKey("far_goal_unknown_weight")) {
    opts->set_far_goal_unknown_weight(dict->GetDouble("far_goal_unknown_weight"));
  }
  if (dict->HasKey("publish_vg_markers")) {
    opts->set_publish_vg_markers(dict->GetBool("publish_vg_markers"));
  }
  if (dict->HasKey("use_planner_costmap")) {
    opts->set_use_planner_costmap(dict->GetBool("use_planner_costmap"));
  }
  if (dict->HasKey("planner_costmap_topic")) {
    opts->set_planner_costmap_topic(dict->GetString("planner_costmap_topic"));
  }
  if (dict->HasKey("planner_costmap_lethal_threshold")) {
    opts->set_planner_costmap_lethal_threshold(
        dict->GetInt("planner_costmap_lethal_threshold"));
  }
  if (dict->HasKey("far_use_trajectory_edges")) {
    opts->set_far_use_trajectory_edges(dict->GetBool("far_use_trajectory_edges"));
  }
  if (dict->HasKey("far_trajectory_node_count")) {
    opts->set_far_trajectory_node_count(dict->GetInt("far_trajectory_node_count"));
  }
}

}  // namespace

proto::ExplorationOptions DefaultOptions() {
  proto::ExplorationOptions options;
  options.set_map_frame("map");
  options.set_default_area_half_extent_m(20.0);
  options.set_completion_coverage_ratio(0.9);
  options.set_grid_cell_size_m(4.0);
  options.set_grid_cols(21);
  options.set_grid_rows(21);
  options.set_grid_nearby_radius(2);
  options.set_sensor_range_m(8.0);
  options.set_camera_height_m(0.6);
  options.set_depth_min_m(0.1);
  options.set_depth_max_m(8.0);
  options.set_depth_stride(4);
  options.set_ground_height_tol_m(0.15);
  options.set_costmap_resolution_m(0.1);
  options.set_costmap_size_cells(400);
  options.set_collision_radius_m(0.35);
  options.set_viewpoint_num_x(20);
  options.set_viewpoint_num_y(20);
  options.set_viewpoint_resolution_m(1.2);
  options.set_viewpoint_yaw_samples(8);
  options.set_keypose_spacing_m(1.0);
  options.set_keypose_connect_dist_m(3.0);
  options.set_lookahead_distance_m(4.5);
  options.set_use_line_of_sight_lookahead(true);
  options.set_min_frontier_cluster_size(5);
  options.set_tsp_exact_max_nodes(12);
  options.set_rush_home(true);
  options.set_at_home_dist_m(0.5);
  options.set_explorer_backend("rgbd_tare");
  options.set_far_connect_dist_m(6.0);
  options.set_far_contour_stride_cells(3);
  options.set_far_attempt_unknown(true);
  options.set_far3d_voxel_dim_m(0.2);
  options.set_far3d_contour_thresh(10);
  options.set_far3d_contour_blur_size(5);
  options.set_far3d_graph_pool_size(256);
  options.set_far3d_edge_finalize_thred(3);
  options.set_far3d_height_margin_m(1.5);
  options.set_far3d_position_filter_m(0.5);
  options.set_far3d_pillar_perimeter_m(2.0);
  options.set_far3d_dumper_thred(8);
  options.set_viewpoint_num_z(3);
  options.set_viewpoint_resolution_z_m(0.5);
  options.set_use_momentum(true);
  options.set_momentum_dist_m(0.5);
  options.set_momentum_thred(3);
  options.set_local_path_optimization_iters(2);
  options.set_use_frontier_points(true);
  options.set_use_terrain_height(true);
  options.set_nogo_half_extent_m(0.0);
  options.set_point_cloud_voxel_m(0.2);
  options.set_point_cloud_min_points_per_voxel(2);
  options.set_lidar_h_res_deg(2.0);
  options.set_lidar_v_res_deg(5.0);
  options.set_tsp_use_two_opt(true);
  options.set_far_is_autoswitch(true);
  options.set_far_viewpoint_extend(true);
  options.set_far_local_planner_range_m(5.0);
  options.set_far_path_momentum_thred(3);
  options.set_far_is_static_env(false);
  options.set_far_converge_dist_m(0.5);
  options.set_far_terrain_grid_res_m(0.25);
  options.set_far_scan_voxel_m(0.2);
  options.set_use_prior_map(false);
  options.set_prior_map_topic("/map");
  options.set_prior_map_lethal_threshold(65);
  options.set_far2d_graph_pool_size(256);
  options.set_far2d_dumper_thred(8);
  options.set_far_goal_distance_weight(1.0);
  options.set_far_goal_unknown_weight(0.1);
  options.set_publish_vg_markers(true);
  options.set_use_planner_costmap(true);
  options.set_planner_costmap_topic("/global_costmap");
  options.set_planner_costmap_lethal_threshold(50);
  options.set_far_use_trajectory_edges(true);
  options.set_far_trajectory_node_count(24);
  options.set_tsp_use_ortools(false);
  options.set_use_lidar_primary(false);
  return options;
}

proto::ExplorationOptions LoadOptions(const std::string& config_directory,
                                      const std::string& relative_path) {
  proto::ExplorationOptions options = DefaultOptions();
  ConfigurationFileResolver resolver({config_directory});
  const std::string code = resolver.GetFileContentOrDie(relative_path);
  auto dict = LuaParameterDictionary::NonReferenceCounted(
      code, std::make_unique<ConfigurationFileResolver>(
                std::vector<std::string>{config_directory}));
  ApplyLua(dict.get(), &options);
  return options;
}

}  // namespace autonomy::perception::exploration

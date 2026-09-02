-- RGB-D + 3D point cloud FAR exploration (contour polygons + persistent VG).

map_frame = "map"
default_area_half_extent_m = 20.0
completion_coverage_ratio = 0.9

explorer_backend = "far3d"

sensor_range_m = 8.0
camera_height_m = 0.6
depth_min_m = 0.1
depth_max_m = 8.0
depth_stride = 4
ground_height_tol_m = 0.15

costmap_resolution_m = 0.1
costmap_size_cells = 400
collision_radius_m = 0.35

lookahead_distance_m = 4.5
use_line_of_sight_lookahead = true
min_frontier_cluster_size = 5

far_connect_dist_m = 6.0
far_attempt_unknown = true

far3d_voxel_dim_m = 0.2
far3d_contour_thresh = 10
far3d_contour_blur_size = 5
far3d_graph_pool_size = 256
far3d_edge_finalize_thred = 3
far3d_height_margin_m = 1.5
far3d_position_filter_m = 0.5
far3d_pillar_perimeter_m = 2.0
far3d_dumper_thred = 8

viewpoint_num_z = 3
viewpoint_resolution_z_m = 0.5
use_momentum = true
local_path_optimization_iters = 2
point_cloud_voxel_m = 0.2

far_is_autoswitch = true
far_viewpoint_extend = true
far_local_planner_range_m = 5.0
far_is_static_env = false
use_prior_map = true
prior_map_topic = "/map"
publish_vg_markers = true
use_planner_costmap = true
planner_costmap_topic = "/global_costmap"
far_goal_distance_weight = 1.0
far_goal_unknown_weight = 0.1

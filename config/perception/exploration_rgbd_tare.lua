-- RGB-D TARE-style hierarchical exploration parameters.

map_frame = "map"
default_area_half_extent_m = 20.0
completion_coverage_ratio = 0.9

grid_cell_size_m = 4.0
grid_cols = 21
grid_rows = 21
grid_nearby_radius = 2

sensor_range_m = 8.0
camera_height_m = 0.6
depth_min_m = 0.1
depth_max_m = 8.0
depth_stride = 4
ground_height_tol_m = 0.15

costmap_resolution_m = 0.1
costmap_size_cells = 400
collision_radius_m = 0.35

viewpoint_num_x = 20
viewpoint_num_y = 20
viewpoint_resolution_m = 1.2
viewpoint_yaw_samples = 8
viewpoint_num_z = 3
viewpoint_resolution_z_m = 0.5
use_momentum = true
local_path_optimization_iters = 2
point_cloud_voxel_m = 0.2
tsp_use_two_opt = true
boundary_ply_path = "perception/boundary.ply"  -- optional

use_prior_map = true
use_planner_costmap = true
publish_vg_markers = true

keypose_spacing_m = 1.0
keypose_connect_dist_m = 3.0

lookahead_distance_m = 4.5
use_line_of_sight_lookahead = true

min_frontier_cluster_size = 5
tsp_exact_max_nodes = 12

rush_home = true
at_home_dist_m = 0.5

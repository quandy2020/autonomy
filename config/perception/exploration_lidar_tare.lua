-- LiDAR-first TARE (registered scan + terrain map, CMU-style inputs).

map_frame = "map"
default_area_half_extent_m = 30.0
completion_coverage_ratio = 0.9
use_lidar_primary = true

grid_cell_size_m = 4.0
grid_cols = 25
grid_rows = 25
grid_nearby_radius = 2

sensor_range_m = 15.0
camera_height_m = 0.6
depth_stride = 2
ground_height_tol_m = 0.15

costmap_resolution_m = 0.15
costmap_size_cells = 400
collision_radius_m = 0.35

viewpoint_num_x = 24
viewpoint_num_y = 24
viewpoint_resolution_m = 1.0
viewpoint_yaw_samples = 8
viewpoint_num_z = 3
viewpoint_resolution_z_m = 0.5
use_momentum = true
use_terrain_height = true
use_frontier_points = true
local_path_optimization_iters = 2
point_cloud_voxel_m = 0.2
tsp_use_two_opt = true

keypose_spacing_m = 1.0
keypose_connect_dist_m = 3.0
lookahead_distance_m = 4.5
use_line_of_sight_lookahead = true

min_frontier_cluster_size = 5
tsp_exact_max_nodes = 12

rush_home = true
at_home_dist_m = 0.5

-- boundary_ply_path = "perception/boundary.ply"

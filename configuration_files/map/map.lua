-- Copyright 2025 The Openbot Authors(duyongquan)
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

AUTONOMY_MAP = {

    use_costmap_2d = true,
    use_costmap_3d = false,
 
    costmap2d = {
        map_file = "turtlebot3_house.yaml",
        -- global_costmap = {
        --     update_frequency = 5.0,
        --     global_frame = "map",
        --     robot_base_frame = "base_link",
        --     resolution = 0.05,
        --     robot_radius = 0.22,
        --     track_unknown_space = true,
        --     plugins = {"static_layer", "obstacle_layer", "inflation_layer"},
        --     obstacle_layer = {
        --         plugin = "nav2_costmap_2d::ObstacleLayer",
        --         enabled = true,
        --         observation_sources = {
        --             sensor_id = "scan"
        --             max_obstacle_height = 2.0
        --             clearing = true,
        --             marking = true,
        --             data_type = "LaserScan",
        --             raytrace_max_range = 3.0,
        --             raytrace_min_range = 0.0,
        --             obstacle_max_range = 2.5,
        --             obstacle_min_range = 0.0,
        --         },
        --     },
        --     static_layer = {
        --         plugin = "nav2_costmap_2d::StaticLayer",
        --         map_subscribe_transient_local = true,
        --         always_send_full_costmap = true,
        --     },
        --     inflation_layer = {
        --         plugin = "nav2_costmap_2d::InflationLayer",
        --         cost_scaling_factor = 3.0,
        --         inflation_radius = 0.7,
        --     },
        --     always_send_full_costmap = true,
        -- },

        -- local_costmap = {
        --     update_frequency = 5.0,
        --     global_frame = "odom",
        --     robot_base_frame = "base_link",
        --     rolling_window = true,
        --     width = 3,
        --     height = 3,
        --     resolution = 0.05,
        --     robot_radius = 0.22,
        --     plugins = {"voxel_layer", "inflation_layer"},
        --     inflation_layer = {
        --         plugin = "nav2_costmap_2d::InflationLayer",
        --         cost_scaling_factor = 3.0,
        --         inflation_radius = 0.70,
        --     }
               
        --     voxel_layer = {
        --         plugin = "nav2_costmap_2d::VoxelLayer",
        --         enabled = true,
        --         publish_voxel_map = true,
        --         origin_z = 0.0,
        --         z_resolution = 0.05,
        --         z_voxels = 16,
        --         max_obstacle_height = 2.0,
        --         mark_threshold = 0,
        --         observation_sources = {
        --             sensor_id = "scan",
        --             max_obstacle_height = 2.0,
        --             clearing = true,
        --             marking = true,
        --             data_type = "LaserScan",
        --             raytrace_max_range = 3.0,
        --             raytrace_min_range = 0.0,
        --             obstacle_max_range = 2.5,
        --             obstacle_min_range = 0.0,
        --         },
        --     },
            
        --     static_layer = {
        --         plugin = "nav2_costmap_2d::StaticLayer",
        --         map_subscribe_transient_local = true,
        --     },
        --     always_send_full_costmap = true,
        -- },
    },

    costmap3d = {
        map_file = "dfdgdg.ply",
    }
}

-- output the map
return AUTONOMY_MAP


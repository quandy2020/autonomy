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


AUTONOMY_PLANNER = {

    expected_planner_frequency = 10.0,

    frame_id = "map",

    -- planner plugins
    plugins = { 
        "NavfnPlanner"
    }, 

    -- NavfnPlanner
    NavfnPlanner = {
        plugin = "planning::NavfnPlanner",
        enabled = true,
        tolerance = 0.2,
        use_astar = true,
        allow_unknown = true,
    },

    -- global costmap --
    costmap = {
        name = "global_costmap",
        resolution = 0.05,
        width = 100,
        height = 100,
        update_frequency = 1.0,
        rolling_window = true,
        robot_radius = 0.5,
        always_send_full_costmap = true,
    
        -- footprint
        footprint = {
            {0.5, 0.5},
            {0.5, -0.5},
            {-0.5, -0.5},
            {-0.5, 0.5},
        },
        footprint_padding = 0.0,

        -- plugins
        plugins = {"static_layer", "inflation_layer"},

        -- static layer
        static_layer = {
            plugin = "map::costmap_2d::StaticLayer",
            enabled = true,
            subscribe_to_updates = false,
            transform_tolerance = 0.1,
            footprint_clearing_enabled = true,
            map_topic = "/map",
        },

        -- inflation layer
        inflation_layer = {
            cost_scaling_factor = 0.5,
            inflation_radius = 0.5,
        },

        -- obstacle layer
        obstacle_layer = {
           plugin = "map::costmap_2d::ObstacleLayer",
           enabled = true,
           footprint_clearing_enabled = true,
           observation_sources = {"scan", "point_cloud"},

           -- scan source
           -- sensor type and params
           scan = {
               topic = "/scan",
               max_obstacle_height = 2.0,
               min_obstacle_height = 0.0,
               clearing = true,
               marking = true,
               data_type = "LaserScan",
               raytrace_min_range = 0.0,
               raytrace_max_range = 10.0,
               obstacle_min_height = 0.0,
               obstacle_max_height = 2.0,
           },

           -- point cloud source
           -- sensor type and params
           point_cloud = {
               topic = "/point_cloud",
               max_obstacle_height = 2.0,
               min_obstacle_height = 0.0,
               clearing = true,
               marking = true,
               data_type = "PointCloud2",
               raytrace_min_range = 0.0,
               raytrace_max_range = 10.0,
               obstacle_min_height = 0.0,
               obstacle_max_height = 2.0,
           },
        },
    }
}
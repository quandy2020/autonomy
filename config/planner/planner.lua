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

    -- NavFn global planner configuration
    navfn_planner = {
        -- Tolerance near the goal point (meters)
        tolerance = 0.1,

        -- Whether to use A* algorithm (false uses Dijkstra)
        use_astar = false,

        -- Whether to allow unknown regions (NO_INFORMATION) in planning
        allow_unknown = true,

        -- Whether to use final approach orientation
        use_final_approach_orientation = false,
    },

    -- Expected planner frequency (Hz), used for internal performance checks (PlannerServer::expected_planner_frequency)
    expected_planner_frequency = 5.0,


    -- Global map configuration (for global path planning)
    costmap = {
        enabled = true,
        -- Default map name (used if name is not specified in costmap configuration)
        name = "global_map",
        frame_id = "map",
        resolution = 0.05,
        width = 20.0,
        height = 20.0,
        update_frequency = 5.0,
        robot_radius = 0.22,
        always_send_full_costmap = true,
        -- Plugin list (executed in order)
        -- Layers: static_layer, obstacle_layer, voxel_layer, range_sensor_layer, denoise_layer, inflation_layer
        -- Filters: keepout_filter, speed_filter, binary_filter
        plugins = {"static_layer", "denoise_layer", "inflation_layer"},
        
        -- Static layer configuration: load static obstacles from SLAM map
        static_layer = {
            plugin = "libautonomy_map_layers_static_layer.so",
            enabled = true,
            subscribe_to_updates = false,        -- Whether to subscribe to map updates
            transform_tolerance = 0.1,           -- Transform tolerance (seconds)
            footprint_clearing_enabled = false,  -- Whether to clear robot footprint area
            map_topic = "map",                   -- Static map topic
        },
        
        -- Denoise layer configuration: filter isolated obstacles caused by noise
        denoise_layer = {
            plugin = "libautonomy_map_layers_denoise_layer.so",
            enabled = true,
            denoise_radius = 2,  -- Denoise radius (pixels), obstacle groups smaller than this size will be removed
        },
        
        -- Inflation layer configuration: inflate obstacles to maintain safe distance
        inflation_layer = {
            plugin = "libautonomy_map_layers_inflation_layer.so",
            enabled = true,
            cost_scaling_factor = 3.0,  -- Cost decay factor (larger values decay faster)
            inflation_radius = 0.55,    -- Inflation radius (meters), should be larger than robot radius
            inflate_unknown = false,    -- Whether to inflate unknown regions
            inflate_around_unknown = false,  -- Whether to inflate around unknown regions
        },
    },
}

-- LuaParameterDictionary expects the script to return a table when loaded directly.
return { AUTONOMY_PLANNER = AUTONOMY_PLANNER }
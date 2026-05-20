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

include "common.lua"

AUTONOMY_PLANNER = {

    navfn_planner = {
        tolerance = 0.1,
        use_astar = false,
        allow_unknown = true,
        use_final_approach_orientation = false,
    },

    dijkstra_planner = {
        tolerance = 0.1,
        allow_unknown = true,
        use_final_approach_orientation = false,
    },

    -- Theta* any-angle grid planner
    theta_star_planner = {
        how_many_corners = 8,
        allow_unknown = true,
        w_euc_cost = 2.0,
        w_traversal_cost = 1.0,
        w_heuristic_cost = 1.0,
        terminal_checking_interval = 5000,
    },

    expected_planner_frequency = 5.0,
    costmap_update_timeout = 5.0,

    planner_plugins = {
        "navfn_planner",
        "dijkstra_planner",
        "theta_star_planner",
    },
    default_planner_id = "navfn_planner",

    -- Optional external planner plugin description XML files (autolink format)
    planner_plugin_libraries = {},

    smoother_plugins = {
        "simple_smoother",
        "savitzky_golay_smoother",
        -- "fem_pos_smoother",
        -- "cos_theta_smoother",
    },
    default_smoother_id = "simple_smoother",

    simple_smoother = {
        tolerance = 1e-10,
        max_iterations = 1000,
        w_data = 0.2,
        w_smooth = 0.3,
        do_refinement = true,
        refinement_num = 2,
        enforce_path_inversion = true,
    },

    savitzky_golay_smoother = {
        do_refinement = true,
        refinement_num = 2,
        enforce_path_inversion = true,
        window_size = 7,
        poly_order = 3,
    },

    -- Optional advanced smoothers (enable in smoother_plugins when needed)
    fem_pos_smoother = {
        weight_fem_pos_deviation = 1.0e5,
        weight_ref_deviation = 1.0,
        weight_path_length = 1.0,
        max_iter = 500,
        time_limit = 0.0,
    },
    fem_pos_smoother_path_bound = 0.5,

    cos_theta_smoother = {
        weight_cos_included_angle = 10000.0,
        weight_anchor_points = 1.0,
        weight_length = 1.0,
    },
    cos_theta_smoother_path_bound = 0.5,

    path_simplify_epsilon = 0.02,
    auto_smooth_after_plan = false,
    auto_smooth_duration = 1.0,

    costmap = {
        enabled = true,
        name = "global_map",
        frame_id = AUTONOMY_COMMON.global_frame,
        resolution = 0.05,
        width = 20.0,
        height = 20.0,
        update_frequency = 5.0,
        robot_radius = 0.22,
        always_send_full_costmap = true,
        -- Global costmap: static map + inflation only (no rolling window).
        -- Dynamic obstacles are handled by the local costmap on the controller.
        -- To add live global obstacles, append "obstacle_layer" and configure it.
        plugins = {"static_layer", "denoise_layer", "inflation_layer"},

        static_layer = {
            plugin = "libautonomy_map_layers_static_layer.so",
            enabled = true,
            subscribe_to_updates = false,
            transform_tolerance = 0.1,
            footprint_clearing_enabled = false,
            map_topic = "map",
        },

        denoise_layer = {
            plugin = "libautonomy_map_layers_denoise_layer.so",
            enabled = true,
            denoise_radius = 2,
        },

        inflation_layer = {
            plugin = "libautonomy_map_layers_inflation_layer.so",
            enabled = true,
            cost_scaling_factor = 3.0,
            inflation_radius = 0.55,
            inflate_unknown = false,
            inflate_around_unknown = false,
        },
    },
}

return { AUTONOMY_PLANNER = AUTONOMY_PLANNER }

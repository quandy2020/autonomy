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
if AUTONOMY_COMMON == nil then
  include "common.lua"
end

AUTONOMY_CONTROLLER = {
    -- Must match 1/model_dt (0.05 -> 20 Hz) for MPPI control-sequence shifting.
    controller_frequency = 20.0,
    -- Allow short TF/control hiccups without tearing down FollowPath.
    -- Allow long MPPI laps in controller_test before FollowPath aborts.
    failure_tolerance = 30.0,
    publish_zero_velocity = false,

    controller_plugins = {
        "graceful_controller:GracefulController",
        "mppi_controller:MppiController",
    },
    controller_plugin_libraries = {},

    -- Shared with navigator.lua / BT GoalReached (see config/common.lua).
    goal_checker = {
        xy_goal_tolerance = AUTONOMY_COMMON.goal_reached_tolerance,
        -- Keep heading convergence stricter than XY tolerance at goal.
        yaw_goal_tolerance = 0.35,
        stateful = true,
    },
    progress_checker = {
        required_movement_radius = 0.5,
        movement_time_allowance = 10.0,
    },

    -- Local costmap disabled in attached mode; TaskScheduler shares planner global costmap.
    costmap = {
        enabled = false,
        name = "local_costmap",
        frame_id = AUTONOMY_COMMON.global_frame,
        resolution = 0.05,
        width = 3.0,
        height = 3.0,
        robot_radius = 0.22,
        -- Rectangle footprint (x: forward, y: left), counter-clockwise.
        footprint = {
            {x = 0.18, y = 0.14},
            {x = 0.18, y = -0.14},
            {x = -0.18, y = -0.14},
            {x = -0.18, y = 0.14},
        },
        update_frequency = 30.0,
        rolling_window = true,
        publish_frequency = 30.0,
        plugins = {"obstacle_layer", "denoise_layer", "inflation_layer"},
        
        -- 障碍物层配置：使用传感器数据检测动态障碍物
        obstacle_layer = {
            plugin = "libautonomy_map_layers_obstacle_layer.so",
            enabled = true,
            footprint_clearing_enabled = true,   -- 清除机器人足迹区域的障碍物
            sensor_sources = {
                pointcloud = {
                    topic = "/points2",
                    data_type = "PointCloud2",
                    max_obstacle_height = 2.0,
                    min_obstacle_height = 0.0,
                    obstacle_min_height = 0.0,
                    obstacle_max_height = 2.0,
                },
            },
        },
        
        -- 降噪层配置：过滤噪声导致的孤立障碍物
        denoise_layer = {
            plugin = "libautonomy_map_layers_denoise_layer.so",
            enabled = true,
            denoise_radius = 2,  -- 降噪半径（像素）
        },
        
        -- 膨胀层配置：膨胀障碍物以保持安全距离
        inflation_layer = {
            plugin = "libautonomy_map_layers_inflation_layer.so",
            enabled = true,
            cost_scaling_factor = 3.0,  -- 代价衰减因子
            inflation_radius = 0.55,    -- 膨胀半径（米）
            inflate_unknown = false,    -- 是否膨胀未知区域
            inflate_around_unknown = false,  -- 是否在未知区域周围膨胀
        },
        
        -- 体素层配置（可选）：3D障碍物检测
        -- voxel_layer = {
        --     plugin = "libautonomy_map_layers_voxel_layer.so",
        --     enabled = false,
        --     footprint_clearing_enabled = true,
        --     sensor_sources = {...},
        -- },
        
        -- 范围传感器层配置（可选）：用于超声波/红外传感器
        -- range_sensor_layer = {
        --     plugin = "libautonomy_map_layers_range_sensor_layer.so",
        --     enabled = false,
        -- },
    },

    graceful_controller = {
        transform_tolerance = AUTONOMY_COMMON.transform_tolerance,
        -- Reduce corner-cutting in narrow indoor maps.
        max_lookahead = 0.55,
        min_lookahead = 0.25,
        v_linear_max = 0.5,
        v_linear_min = 0.05,
        v_angular_max = 1.0,
        slowdown_radius = 0.5,
        initial_rotation = true,
        allow_backward = false,
        -- Enforce obstacle-aware tracking to avoid corner-cutting through walls.
        use_collision_detection = true,
    },

    mppi_controller = {
        -- Progress checker plugin
        progress_checker = {
            plugin = "nav2_controller::SimpleProgressChecker",
            required_movement_radius = 0.5,
            movement_time_allowance = 10.0,
        },
        -- Goal checker plugin (legacy path; prefer top-level goal_checker above)
        goal_checker = {
            plugin = "nav2_controller::SimpleGoalChecker",
            xy_goal_tolerance = AUTONOMY_COMMON.goal_reached_tolerance,
            yaw_goal_tolerance = AUTONOMY_COMMON.goal_reached_tolerance,
            stateful = true,
        },
        name = "mppi_controller",
        plugin = "nav2_mppi_controller::MPPIController",
        time_steps = 56,
        model_dt = 0.05,
        batch_size = 2000,
        vx_std = 0.2,
        vy_std = 0.2,
        wz_std = 0.4,
        vx_max = 0.5,
        vx_min = -0.35,
        vy_max = 0.5,
        wz_max = 1.9,
        iteration_count = 1,
        temperature = 0.3,
        gamma = 0.015,
        motion_model = "DiffDrive",
        visualize = true,
        TrajectoryVisualizer = {
            trajectory_step = 5,
            time_step = 3,
        },
        AckermannConstraints = {
            min_turning_r = 0.2,
        },
        critics = {
            "ConstraintCritic", 
            "CostCritic", 
            "GoalCritic", 
            "GoalAngleCritic",
            "PathAlignCritic", 
            "PathFollowCritic", 
            "PathAngleCritic", 
            "PreferForwardCritic",
        },
        ConstraintCritic = {
            enabled = true,
            cost_power = 1,
            cost_weight = 4.0,
        },
        GoalCritic = {
            enabled = true,
            cost_power = 1,
            cost_weight = 5.0,
            threshold_to_consider = 1.4,
        },
        GoalAngleCritic = {
            enabled = true,
            cost_power = 1,
            cost_weight = 3.0,
            threshold_to_consider = 0.5,
        },
        PreferForwardCritic = {
            enabled = true,
            cost_power = 1,
            cost_weight = 5.0,
            threshold_to_consider = 0.5,
        },
        CostCritic = {
            enabled = true,
            cost_power = 1,
            cost_weight = 3.81,
            critical_cost = 300.0,
            -- Synthetic / layer-less costmaps have no inflation; point cost only.
            consider_footprint = false,
            collision_cost = 1000000.0,
            near_goal_distance = 1.0,
            trajectory_point_step = 2,
        },
        PathAlignCritic = {
            enabled = true,
            cost_power = 1,
            cost_weight = 14.0,
            max_path_occupancy_ratio = 0.05,
            trajectory_point_step = 4,
            threshold_to_consider = 0.5,
            offset_from_furthest = 20,
            use_path_orientations = false,
        },
        PathFollowCritic = {
            enabled = true,
            cost_power = 1,
            cost_weight = 5.0,
            offset_from_furthest = 5,
            threshold_to_consider = 1.4,
        },
        PathAngleCritic = {
            enabled = true,
            cost_power = 1,
            cost_weight = 2.0,
            offset_from_furthest = 4,
            threshold_to_consider = 0.5,
            max_angle_to_furthest = 1.0,
            forward_preference = true,
        },
        ObstaclesCritic = {
            enabled = false, 
            cost_power = 1, 
            repulsion_weight = 1.5, 
            critical_weight = 20.0,
            consider_footprint = false, 
            collision_cost = 10000.0,
            collision_margin_distance = 0.1, 
            near_goal_distance = 0.5,
        },
        VelocityDeadbandCritic = {
            enabled = false, 
            cost_power = 1, 
            cost_weight = 35.0,
            deadband_velocities = {
                0.05, 
                0.05, 
                0.05
            },
        },
        TwirlingCritic = {
            enabled = false, 
            twirling_cost_power = 1, 
            twirling_cost_weight = 10.0,
        },
    },

}

-- LuaParameterDictionary expects the script to return a table when loaded directly.
return { AUTONOMY_CONTROLLER = AUTONOMY_CONTROLLER }
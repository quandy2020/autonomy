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

AUTONOMY_CONTROLLER = {

    -- -- dwb controller
    -- dwb_controller = {

    -- },

    -- -- graceful controller
    -- graceful_controller = {

    -- },

    -- -- MMPI controller
    -- mppi_controller = {

    -- },

    -- -- pure_pursuit controller
    -- pure_pursuit_controller = {

    -- },

    -- -- ROS TEB Local planner 
    -- teb_controller = {

    -- }


    -- plugins = {
    --     "dwb_controller", "graceful_controller", "mppi_controller", "pure_pursuit_controller", "teb_controller",
    -- },

    -- dwb_controller = {
    --     enabled = true,
    -- },
    -- graceful_controller = {
    --     enabled = true,
    -- },
    -- mppi_controller = {
    --     enabled = true,
    -- },
    -- pure_pursuit_controller = {
    --     enabled = true,
    -- },
    -- teb_controller = {
    --     enabled = true,
    -- },

    -- 局部地图配置（用于局部避障）- 可选
    -- 跟随机器人移动的滚动窗口地图，用于局部避障
    -- 可以是 Costmap2D、Costmap3D 或 GridMap 其中之一
    costmap_2d = {
        -- 地图名称
        map_name = "local_costmap",
        
        -- 地图分辨率（米/像素）
        resolution = 0.05,
        
        -- 地图宽度（米），默认 3.0
        width = 3.0,
        
        -- 地图高度（米），默认 3.0
        height = 3.0,
        
        -- 坐标系ID（通常是 "odom" 或 "base_link"）
        frame_id = "odom",
        
        -- 机器人基础坐标系（通常是 "base_link"）
        robot_base_frame = "base_link",
        
        -- 机器人半径（米）
        robot_radius = 0.22,
        
        -- 更新频率（Hz）
        update_frequency = 30.0,
        
        -- 是否启用滚动窗口（rolling window），局部地图应该启用
        rolling_window = true,
        
        -- 是否自动启动地图更新
        auto_start = true,
        
        -- 地图发布主题（用于局部避障地图发布，默认: "local_costmap"）
        map_topic = "local_costmap",
        
        -- 地图发布频率（Hz，默认: 30.0）
        publish_frequency = 30.0,
        
        -- 传感器配置列表（用于地图构建和更新）
        sensors = {
            -- 激光扫描传感器配置
            {
                sensor_id = "laser_scan",
                type = 0,  -- LASER_SCAN = 0
                topic = "/scan",
                frame_id = "laser_frame",
                map_name = "",  -- 如果为空则使用 map_name (即 "local_map")
            },
            -- 点云传感器配置 (PointCloud2)
            {
                sensor_id = "pointcloud2",
                type = 1,  -- POINT_CLOUD2 = 1
                topic = "/points2",
                frame_id = "velodyne",
                map_name = "",  -- 如果为空则使用 map_name (即 "local_map")
            },
        },
        
        -- 地图类型选择（三选一，必须设置其中一个）
        -- 选项1: Costmap2D (2D代价地图)
        costmap_2d = {
            enabled = true,
            -- 插件列表（按顺序执行）
            -- 局部地图通常不需要 static_layer，因为使用滚动窗口
            plugins = {"obstacle_layer", "denoise_layer", "inflation_layer"},
            
            -- 障碍物层配置：使用传感器数据检测动态障碍物
            obstacle_layer = {
                plugin = "libautonomy_map_layers_obstacle_layer.so",
                enabled = true,
                footprint_clearing_enabled = true,   -- 清除机器人足迹区域的障碍物
                sensor_sources = {
                    -- 激光雷达传感器
                    laser_scan = {
                        topic = "/scan",
                        data_type = "LaserScan",
                        max_obstacle_height = 2.0,
                        min_obstacle_height = 0.0,
                        obstacle_min_height = 0.0,
                        obstacle_max_height = 2.0,
                        clearing = true,
                        marking = true,
                        raytrace_min_range = 0.0,
                        raytrace_max_range = 3.0,
                    },
                    -- 点云传感器
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
    }
}
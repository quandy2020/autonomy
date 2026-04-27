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

-- 可视化服务器配置
AUTONOMY_VISUALIZATION = {
    -- 服务器配置
    server_name = "AutonomyViewer",
    host = "0.0.0.0",
    port = 8765,
    
    -- 功能开关
    enable_client_publish = true,      -- 允许 Foxglove Studio 发布消息
    enable_connection_graph = true,    -- 启用连接图功能
    enable_autolink = true,            -- 启用 autolink 订阅
    
    -- Topic 订阅列表（需要转发到 Foxglove 的 topic）
    -- message_type 支持的值：
    --   sensor_msgs: "LaserScan", "PointCloud2", "PointCloud", "Imu", "Range", "Image", "CompressedImage"
    --   planning_msgs: "Path", "Odometry"
    --   map_msgs: "OccupancyGrid"
    --   geometry_msgs: "PoseStamped", "PoseArray", "TransformStamped"
    --   visualization_msgs: "Marker", "MarkerArray"
    topic_subscriptions = {
        {
            topic_name = "/sensor/lidar",
            message_type = "LaserScan",
            enabled = true,
        },
        {
            topic_name = "/planning/path",
            message_type = "Path",
            enabled = true,
        },
        {
            topic_name = "/localization/odometry",
            message_type = "Odometry",
            enabled = true,
        },
        {
            topic_name = "/map",
            message_type = "OccupancyGrid",
            enabled = true,
        },
        {
            topic_name = "/sensor/camera",
            message_type = "Image",
            enabled = true,
        },
        {
            topic_name = "/sensor/pointcloud",
            message_type = "PointCloud2",
            enabled = false,  -- 暂时禁用
        },
        {
            topic_name = "/sensor/imu",
            message_type = "Imu",
            enabled = true,
        },
        {
            topic_name = "/visualization/markers",
            message_type = "MarkerArray",
            enabled = true,
        },
        {
            topic_name = "/localization/pose",
            message_type = "PoseStamped",
            enabled = true,
        },
    },
}

return AUTONOMY_VISUALIZATION

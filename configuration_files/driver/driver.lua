-- Copyright 2025 The Openbot Authors (duyongquan)
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

driver = {
    -- 是否自动启动传感器订阅
    auto_start = true,
    
    -- 默认传感器数据转发目标（可选，如 "map", "localization" 等）
    default_forward_targets = {"map", "localization"},
    
    -- 激光雷达配置列表
    lidars = {
        {
            sensor_id = "front_laser",
            type = 0,  -- 0: LASER_SCAN, 1: POINT_CLOUD2, 2: POINT_CLOUD, 3: MULTI_ECHO_LASER_SCAN
            topic = "scan",
            frame_id = "laser_link",
            enabled = true,
            min_range = 0.1,  -- 米
            max_range = 30.0,  -- 米
            min_angle = -3.14159,  -- 弧度（-180度）
            max_angle = 3.14159,   -- 弧度（180度）
            forward_targets = {"map", "localization"}
        },
        -- 可以添加更多激光雷达
        -- {
        --     sensor_id = "rear_laser",
        --     type = 0,
        --     topic = "rear_scan",
        --     frame_id = "rear_laser_link",
        --     enabled = false,
        --     min_range = 0.1,
        --     max_range = 30.0,
        --     min_angle = -3.14159,
        --     max_angle = 3.14159,
        --     forward_targets = {"map"}
        -- }
    },
    
    -- IMU配置列表
    imus = {
        {
            sensor_id = "imu",
            topic = "imu",
            frame_id = "imu_link",
            enabled = true,
            angular_velocity_covariance_scale = 1.0,
            linear_acceleration_covariance_scale = 1.0,
            orientation_covariance_scale = 1.0,
            forward_targets = {"localization"},
            sampling_rate = 100.0  -- Hz
        }
    },
    
    -- 相机配置列表
    cameras = {
        {
            sensor_id = "front_camera",
            type = 0,  -- 0: MONO, 1: STEREO, 2: RGBD
            image_topic = "camera/image_raw",
            camera_info_topic = "camera/camera_info",
            depth_topic = "",  -- 仅用于RGBD相机
            frame_id = "camera_link",
            enabled = true,
            width = 640,  -- 像素，0表示使用原始尺寸
            height = 480,  -- 像素，0表示使用原始尺寸
            encoding = "rgb8",
            forward_targets = {"perception"},
            sampling_rate = 30.0  -- Hz
        },
        -- RGBD相机示例（注释掉）
        -- {
        --     sensor_id = "rgbd_camera",
        --     type = 2,  -- RGBD
        --     image_topic = "camera/rgb/image_raw",
        --     camera_info_topic = "camera/rgb/camera_info",
        --     depth_topic = "camera/depth/image_raw",
        --     frame_id = "camera_link",
        --     enabled = false,
        --     width = 640,
        --     height = 480,
        --     encoding = "rgb8",
        --     forward_targets = {"perception", "map"},
        --     sampling_rate = 30.0
        -- }
    },
    
    -- 测距传感器配置列表
    ranges = {
        {
            sensor_id = "ultrasonic_front",
            topic = "ultrasonic/front",
            frame_id = "ultrasonic_front_link",
            enabled = true,
            radiation_type = 0,  -- 0: ULTRASOUND, 1: INFRARED
            field_of_view = 0.174533,  -- 弧度（约10度）
            min_range = 0.02,  -- 米
            max_range = 4.0,   -- 米
            forward_targets = {"map"},
            sampling_rate = 10.0  -- Hz
        },
        -- 可以添加更多测距传感器
        -- {
        --     sensor_id = "infrared_rear",
        --     topic = "infrared/rear",
        --     frame_id = "infrared_rear_link",
        --     enabled = false,
        --     radiation_type = 1,  -- INFRARED
        --     field_of_view = 0.087266,  -- 弧度（约5度）
        --     min_range = 0.05,
        --     max_range = 2.0,
        --     forward_targets = {"map"},
        --     sampling_rate = 20.0
        -- }
    },
    
    -- GPS/导航卫星配置列表
    gps_sensors = {
        {
            sensor_id = "gps",
            topic = "fix",
            frame_id = "gps_link",
            enabled = false,  -- 默认禁用，因为需要硬件支持
            min_position_accuracy = 1.0,  -- 米
            min_altitude_accuracy = 2.0,   -- 米
            forward_targets = {"localization"},
            sampling_rate = 1.0,  -- Hz
            use_for_localization = true,
            service_type = 1  -- 位掩码：1=GPS, 2=GLONASS, 4=COMPASS, 8=GALILEO
        }
    }
}


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

-- AMCL Localization Options
-- Configuration parameters for the AMCL (Adaptive Monte Carlo Localization) algorithm
-- 对应 proto: autonomy.localization.proto.AmclOptions

AMCL_OPTIONS = {
    -- Motion Model Parameters
    -- Error parameters for the motion model (odometry noise model)
    -- 这些参数定义了里程计运动模型的噪声特性
    alpha1 = 0.2,              -- Rotation error from rotation
    alpha2 = 0.2,              -- Rotation error from translation
    alpha3 = 0.2,              -- Translation error from translation
    alpha4 = 0.2,              -- Translation error from rotation
    alpha5 = 0.2,              -- Translation error from translation (for omni-directional robots)
    
    -- Robot motion model type
    -- 0 = DIFFERENTIAL (差速驱动机器人)
    -- 1 = OMNI (全向机器人)
    -- 2 = DIFFERENTIAL_MOTION_MODEL (nav2_amcl::DifferentialMotionModel)
    robot_model_type = 2,     -- 默认使用 DIFFERENTIAL_MOTION_MODEL
    
    -- Particle Filter Parameters
    -- 粒子滤波器参数
    min_particles = 500,       -- Minimum number of particles
    max_particles = 2000,     -- Maximum number of particles
    pf_err = 0.01,            -- Error threshold for particle filter
    pf_z = 0.99,              -- Z-score for particle filter
    recovery_alpha_fast = 0.0, -- Fast recovery alpha (for KLD sampling)
    recovery_alpha_slow = 0.0, -- Slow recovery alpha (for KLD sampling)
    resample_interval = 1,     -- Interval between resampling (1 = every update)
    
    -- Laser Sensor Model Parameters
    -- 激光传感器模型参数
    -- 0 = BEAM (Beam model)
    -- 1 = LIKELIHOOD_FIELD (Likelihood field model)
    -- 2 = LIKELIHOOD_FIELD_PROB (Likelihood field probability model)
    laser_model_type = 2,     -- 默认使用 LIKELIHOOD_FIELD_PROB
    
    -- Laser range parameters
    laser_max_range = -1.0,   -- Maximum range for laser readings (m, -1.0 = use sensor default)
    laser_min_range = -1.0,   -- Minimum range for laser readings (m, -1.0 = use sensor default)
    max_beams = 60,           -- Maximum number of beams to use from laser scan
    laser_likelihood_max_dist = 2.0,  -- Maximum distance for likelihood field (m)
    
    -- Laser sensor model weights
    -- 激光传感器模型权重
    z_hit = 0.95,             -- Weight for hit probability
    z_rand = 0.05,            -- Weight for random probability
    z_max = 0.05,             -- Weight for max range probability
    z_short = 0.1,            -- Weight for short range probability
    sigma_hit = 0.2,          -- Standard deviation for hit probability
    lambda_short = 0.1,       -- Exponential decay parameter for short range
    
    -- Beam skipping parameters (for likelihood_field_prob model)
    -- 光束跳过参数（用于 likelihood_field_prob 模型）
    do_beamskip = false,      -- Enable beam skipping
    beam_skip_distance = 0.5, -- Distance threshold for beam skipping
    beam_skip_threshold = 0.3, -- Threshold for beam skipping
    beam_skip_error_threshold = 0.9,  -- Error threshold for beam skipping
    
    -- Frame IDs
    -- 坐标系ID
    base_frame_id = "base_link",     -- Base frame of the robot
    global_frame_id = "map",         -- Global frame (map frame)
    odom_frame_id = "odom",          -- Odometry frame
    
    -- Update Thresholds
    -- 更新阈值
    update_min_a = 0.2,       -- Minimum angular change to trigger update (rad)
    update_min_d = 0.25,      -- Minimum linear change to trigger update (m)
    
    -- Map Parameters
    -- 地图参数
    map_topic = "/map",       -- Topic name for map messages
    first_map_only = false,   -- Only use the first map received
    freespace_downsampling = false,  -- Downsample free space for uniform sampling
    
    -- Initial Pose Parameters
    -- 初始位姿参数
    set_initial_pose = false, -- Set initial pose from parameters
    initial_pose = {
        x = 0.0,              -- Initial x position (m)
        y = 0.0,              -- Initial y position (m)
        z = 0.0,              -- Initial z position (m)
        yaw = 0.0,            -- Initial yaw angle (rad)
    },
    always_reset_initial_pose = false,  -- Always reset initial pose on startup
    
    -- Transform Parameters
    -- 变换参数
    tf_broadcast = true,      -- Broadcast transform from map to odom
    transform_tolerance = 1.0, -- Transform tolerance (s)
    
    -- Topic Names
    -- 话题名称
    scan_topic = "/scan",     -- Topic name for laser scan messages
    
    -- Other Parameters
    -- 其他参数
    save_pose_rate = 20.0,   -- Rate to save pose (Hz) - 用于 TF 发布频率
    introspection_mode = "",  -- Introspection mode (for debugging)
    allow_parameter_qos_overrides = false,  -- Allow QoS parameter overrides
}


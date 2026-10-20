/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file livo_odometry_config.hpp
 * @brief LivoOdometryConfig POD (no orchestrator / protobuf I/O types).
 */

#ifndef ARGUS_ODOMETRY_LIVO_ODOMETRY_CONFIG_HPP_
#define ARGUS_ODOMETRY_LIVO_ODOMETRY_CONFIG_HPP_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "argus/frontend/dynamic_filter_base.hpp"
#include "argus/frontend/types.hpp"
#include "argus/fusion/confidence_gating.hpp"
#include "argus/fusion/intensity_assist.hpp"
#include "argus/relocalization/loop_closure/manager.hpp"
#include "argus/map/map_self_cleaning_config.hpp"
#include "argus/map/voxel_map.hpp"
#include "argus/mapping/map_builder.hpp"
#include "argus/perception/degeneracy_detector.hpp"
#include "argus/relocalization/manager.hpp"

namespace argus {
namespace odometry {

/** @brief Direct photometric VIO (FAST-LIVO2 VIOManager) parameters. */
struct VioConfig {
    /** @brief FAST-LIVO2-style camera YAML (`cam_model`, `cam_fx`, …). */
    std::string camera_calib_file;
    /** @brief LiDAR→camera rotation (row-major 9) and translation (3). */
    std::vector<double> Rcl = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    std::vector<double> Pcl = {0, 0, 0};
    int grid_size = 5;
    int grid_n_width = 17;
    int grid_n_height = 14;
    int patch_size = 8;
    int patch_pyramid_level = 3;
    int max_iterations = 5;
    double outlier_threshold = 1000.0;
    double image_point_cov = 100.0;
    bool enable_normal = true;
    bool enable_inverse_composition = false;
    bool enable_raycast = false;
    bool enable_exposure_estimate = true;
    bool enable_colmap_output = false;
    std::string log_dir;
};

/**
 * @brief Top-level LivoOdometry configuration (sensors, map, loop, reloc).
 *
 * Loaded by `LoadLivoOdometryConfig`; nested structs carry their own docs.
 */
/**
 * @brief Eval-only accumulating pose bias (drift-recovery A/B).
 *        Applied on the estimate thread after each LIO map update.
 */
struct SyntheticDriftConfig {
    bool enable = false;
    /** @brief Extra yaw (rad) per meter of travel. */
    double yaw_rad_per_m = 0.0;
    /** @brief Body +X stretch (m) per meter of travel. */
    double tx_m_per_m = 0.0;
    /** @brief Min travel (m) between bias applications. */
    double step_m = 0.5;
};

struct LivoOdometryConfig {
    frontend::SlamMode slam_mode =
        frontend::kOnlyLio; /**< @brief LIO / VIO / LIVO. */
    bool enable_lidar = true;   /**< @brief Enable LiDAR frontend. */
    bool enable_imu = true;     /**< @brief Enable IMU propagation. */
    bool enable_image = false;    /**< @brief Enable camera / VIO path. */
    double exposure_time_init =
        0.0; /**< @brief Initial inverse exposure guess. */
    double surface_voxel_size =
        0.5;                   /**< @brief Surface feature voxel size (m). */
    map::VoxelMapConfig voxel; /**< @brief Voxel map / plane params. */
    Eigen::Matrix3d ext_R =
        Eigen::Matrix3d::Identity();                 /**< @brief Lidar→IMU R. */
    Eigen::Vector3d ext_T = Eigen::Vector3d::Zero(); /**< @brief Lidar→IMU t. */
    double gyr_cov = 0.1;        /**< @brief Gyro process noise scale. */
    double acc_cov = 0.1;        /**< @brief Accel process noise scale. */
    double inv_expo_cov = 0.2;   /**< @brief Inverse-exposure process noise. */
    int imu_init_frames = 20;    /**< @brief IMU static init window (frames). */
    bool enable_gravity_estimate = true;  /**< @brief Estimate gravity in init. */
    bool enable_bias_estimate = true;    /**< @brief Estimate accel / gyro bias. */
    bool enable_exposure_estimate = true; /**< @brief Online exposure estimation. */
    bool enable_motion_compensate = true; /**< @brief Deskew LiDAR with IMU. */
    bool enable_gravity_align = false;    /**< @brief Align world z to gravity. */
    std::string imu_log_dir; /**< @brief Optional IMU debug dump dir. */

    /** @brief Visual ESIKF; used when `enable_image` and camera calib is available.
     */
    VioConfig vio;

    /** @brief Scene degeneracy (tunnel / corridor FIM); default on. */
    bool enable_degeneracy = true;
    perception::DegeneracyConfig
        degeneracy; /**< @brief Degeneracy detector knobs. */

    frontend::DynamicFilterFactoryConfig
        dynamic_filter; /**< @brief Dynamic object filter. */

    bool enable_confidence_gating =
        false; /**< @brief Gate residuals by confidence. */
    fusion::ConfidenceGatingConfig confidence_gating;
    double sensor_timeout_sec = 0.5; /**< @brief Stale-sensor timeout (s). */

    loop_closure::LoopClosureConfig
        loop_closure; /**< @brief Loop closure / ScanContext. */

    /** @brief Online MapBuilder package (map_meta / map.xyz / prior.xyz / …).
     */
    bool enable_mapping = true;
    mapping::MapBuilderConfig
        mapping;                    /**< @brief Map package writer settings. */
    std::string mapping_output_dir; /**< @brief Directory for map packages. */

    map::MapSelfCleaningConfig
        map_cleaning; /**< @brief Online voxel pruning. */

    bool enable_intensity_assist =
        true; /**< @brief Intensity-aided residuals. */
    fusion::IntensityAssistConfig intensity_assist;

    /** @brief Isolated BnB pool; results drained on estimate thread. */
    relocalization::RelocalizationConfig relocalization;

    /** @brief Synthetic odometry drift for loop recovery benchmarks. */
    SyntheticDriftConfig synthetic_drift;
};

}  // namespace odometry
}  // namespace argus

#endif  // ARGUS_ODOMETRY_LIVO_ODOMETRY_CONFIG_HPP_

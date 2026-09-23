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
 * @file config.hpp
 * @brief Atlas runtime config: YAML load/save aligned with ORB-SLAM3 Settings.
 *
 * Supports nested `schema="atlas"` keys and flat `schema="orb"` ORB keys;
 * camera / IMU / ORB extractor field names follow ORB-SLAM3 `Settings` where
 * practical.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_COMMON_CONFIG_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_COMMON_CONFIG_HPP_

#include <ostream>
#include <string>
#include <vector>

#include "autonomy/localization/atlas/common/types.hpp"
#include "autonomy/localization/atlas/frontend/feature/orb/orb_params.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * @enum autonomy::localization::atlas::CameraModelType
 * @brief Config-layer camera model tag (ORB Settings::CameraType + Atlas extensions).
 *
 * Used for YAML parsing; actual projection is implemented by
 * `sensor::GeometricCamera` subclasses.
 */
/**
 * @enum autonomy::localization::atlas::CameraSensor
 * @brief Rig used by the frontend factory (mono / stereo / RGB-D).
 *
 * YAML key `camera.sensor`. IMU frontends add IMU on top of this rig.
 */
enum class CameraSensor {
    kMonocular = 0,  ///< Single camera.
    kStereo = 1,     ///< Stereo pair.
    kRgbd = 2,       ///< RGB-D.
};

enum class CameraModelType {
    kPinhole = 0,          ///< Pinhole (no distortion).
    kRectified = 1,        ///< Rectified stereo (treated as pinhole).
    kKannalaBrandt = 2,    ///< Kannala–Brandt equidistant fisheye.
    kRadTan = 3,           ///< Brown–Conrady radial–tangential distortion.
    kFov = 4,              ///< Devernay–Faugeras FOV.
    kUcm = 5,              ///< Mei unified omnidirectional camera (UCM).
    kEucm = 6,             ///< Extended unified camera model (EUCM).
    kDoubleSphere = 7,     ///< Usenko double sphere.
    kEquirectangular = 8,  ///< Equirectangular panorama.
    kRadialDivision = 9,   ///< Fitzgibbon division model.
};

/**
 * @struct autonomy::localization::atlas::AtlasConfig
 * @brief Full-system Atlas runtime configuration snapshot.
 *
 * Typical flow: `LoadConfig(path, &cfg)` → build camera / Tracker / backend from
 * fields → optionally `SaveConfig` / `DumpConfig` for disk or debug.
 */
struct AtlasConfig {
    FrontendMode mode = FrontendMode::kVio;  ///< Frontend modality.
    CameraSensor camera_sensor = CameraSensor::kRgbd;  ///< Rig: mono / stereo / RGB-D.
    FusionStyle fusion = FusionStyle::kLoose;  ///< Loose vs tight fusion.
    BackendType backend = BackendType::kIekf;  ///< Backend type.

    bool vo_enabled = false;  ///< Enable pure visual path.
    bool vio_enabled = true;  ///< Enable VIO.
    bool lio_enabled = false;  ///< Enable LIO.

    // --- Camera1 (ORB Camera1.*) ---
    CameraModelType camera_type = CameraModelType::kPinhole;  ///< Camera1 model.
    double camera_fx = 320.0;  ///< Focal length fx (pixels).
    double camera_fy = 320.0;  ///< Focal length fy (pixels).
    double camera_cx = 320.0;  ///< Principal point cx (pixels).
    double camera_cy = 240.0;  ///< Principal point cy (pixels).
    //! OpenCV distortion (pinhole k1,k2,p1,p2[,k3]) or KannalaBrandt k1..k4.
    std::vector<double> camera_distortion;
    int image_width = 0;  ///< Raw image width; 0 = unknown.
    int image_height = 0;  ///< Raw image height; 0 = unknown.
    int image_new_width = 0;   ///< Resize target width; 0 = no resize.
    int image_new_height = 0;  ///< Resize target height; 0 = no resize.
    float fps = 30.f;  ///< Nominal camera frame rate.
    bool rgb = true;  ///< `Camera.RGB`: true=RGB, false=BGR.
    bool need_undistort = false;  ///< Whether explicit undistortion is needed.
    bool need_rectify = false;  ///< Whether stereo rectification is needed.

    // Stereo / RGB-D
    double camera_baseline_meters = 0.12;  ///< Stereo baseline (meters).
    //! bf = fx * baseline (ORB Camera.bf); if >0 overrides baseline×fx.
    double camera_bf = 0.0;
    //! Depth-map `convertTo` scale (TUM often 1/5000 ≈ 0.0002).
    double depth_map_factor = 1.0;
    //! Near/far point threshold (meters × baseline units, ORB ThDepth).
    float depth_threshold = 40.f;
    float th_far_points = 0.f;  ///< ORB System.thFarPoints; 0 = unused.

    // Camera2 (stereo right; optional)
    double camera2_fx = 0.0;  ///< Right fx; 0 = not configured.
    double camera2_fy = 0.0;  ///< Right fy.
    double camera2_cx = 0.0;  ///< Right cx.
    double camera2_cy = 0.0;  ///< Right cy.
    std::vector<double> camera2_distortion;  ///< Right distortion coeffs.
    //! Row-major 4x4 `T_left_right` (ORB Camera1.T_c1_c2 / Stereo.T_c1_c2).
    std::vector<double> T_c1_c2;
    //! Fisheye stereo overlapping column range (ORB Camera*.overlappingBegin/End).
    int camera_overlapping_begin = 0;
    int camera_overlapping_end = 1000;
    int camera2_overlapping_begin = 0;
    int camera2_overlapping_end = 1000;

    // --- IMU (ORB IMU.*) ---
    double imu_accel_noise = 0.1;  ///< Accelerometer noise density.
    double imu_gyro_noise = 0.01;  ///< Gyroscope noise density.
    double imu_accel_bias_random_walk = 0.001;  ///< Accel bias random walk.
    double imu_gyro_bias_random_walk = 0.0001;  ///< Gyro bias random walk.
    float imu_frequency = 200.f;  ///< Nominal IMU rate (Hz).
    bool insert_kfs_when_lost = true;  ///< Still insert keyframes when lost.
    //! Row-major 4x4 `T_body_camera` (ORB IMU.T_b_c1).
    std::vector<double> T_b_c;

    // --- ORB extractor ---
    feature::OrbParams orb;  ///< ORB feature extraction parameters.

    // --- Atlas map IO (ORB File.loadAtlasFrom / File.saveAtlasTo) ---
    std::string atlas_load_file;  ///< Map path to load at start; empty = skip.
    std::string atlas_save_file;  ///< Map path to save on exit; empty = skip.

    // Backend / map
    double sync_tolerance_ms = 20.0;  ///< Multi-sensor time sync tolerance (ms).
    int max_local_map_points = 50000;  ///< Cap on local map points.
    double voxel_size = 0.2;  ///< Point-cloud voxel size (m).
    int window_size = 10;  ///< Sliding-window keyframe count.
    int ceres_max_iterations = 15;  ///< Ceres max iterations.
    double ceres_pose_weight = 10.0;  ///< Ceres pose residual weight.
    double ceres_imu_weight = 1.0;  ///< Ceres IMU residual weight.
    double ceres_visual_weight = 1.0;  ///< Ceres visual residual weight.

    //! FBoW ORB vocabulary path (.fbow); empty tries default conf path.
    std::string vocabulary_path;

    std::string platform_name = "default";  ///< Platform name (multi-robot config).
    std::string config_path;  ///< Source path of the load (debug).
    //! `"atlas"` nested YAML or `"orb"` flat ORB-SLAM3 keys.
    std::string schema = "atlas";
};

/**
 * @brief Load config from a platform YAML (Atlas nested and/or ORB flat keys).
 * @param path YAML file path.
 * @param[out] out Written on success; must not be nullptr.
 * @return true on success; false if missing file, parse error, or `out==nullptr`.
 *
 * @code{.cpp}
 * AtlasConfig cfg;
 * if (!LoadConfig("/path/to/settings.yaml", &cfg)) {
 *   // handle failure
 * }
 * @endcode
 */
bool LoadConfig(const std::string& path, AtlasConfig* out);

/**
 * @brief Write a full Atlas nested YAML (Settings dump).
 * @param path Destination path.
 * @param cfg Config to write.
 * @return true on success; false on IO error.
 */
bool SaveConfig(const std::string& path, const AtlasConfig& cfg);

/**
 * @brief Human-readable config dump (ORB Settings `operator<<` role).
 * @param[out] os Output stream.
 * @param cfg Config snapshot.
 */
void DumpConfig(std::ostream& os, const AtlasConfig& cfg);

/**
 * @brief Apply Settings-owned fields from `AtlasConfig` to Tracker Options.
 * @param cfg Source config.
 * @param[out] orb Optional; if non-null, copies `cfg.orb`.
 * @param[out] depth_map_factor Optional; if non-null, writes depth scale.
 * @param[out] depth_threshold Optional; if non-null, writes ThDepth.
 * @param[out] rgb Optional; if non-null, writes RGB/BGR flag.
 *
 * @note Any output pointer may be nullptr to skip that field.
 */
inline void ApplyConfigToTrackerOptions(
    const AtlasConfig& cfg,
    feature::OrbParams* orb,
    float* depth_map_factor,
    float* depth_threshold,
    bool* rgb) {
    if (orb != nullptr) {
        *orb = cfg.orb;
    }
    if (depth_map_factor != nullptr) {
        *depth_map_factor = static_cast<float>(cfg.depth_map_factor);
    }
    if (depth_threshold != nullptr) {
        *depth_threshold = cfg.depth_threshold;
    }
    if (rgb != nullptr) {
        *rgb = cfg.rgb;
    }
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_COMMON_CONFIG_HPP_

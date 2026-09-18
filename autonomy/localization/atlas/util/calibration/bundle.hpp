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

#pragma once

//! util/calibration/bundle — one YAML: camera distort + IMU/lidar + extrinsics.

#include "autonomy/localization/atlas/frontend/local_estimator.hpp"
#include "autonomy/localization/atlas/util/calibration/types.hpp"
#include "autonomy/localization/atlas/util/extrinsics.hpp"

#include <string>

#include "yaml-cpp/yaml.h"

namespace autonomy::localization::atlas {
namespace calibration {

/**
 * Full multimodal calibration package.
 *
 * Frame convention (body = IMU unless noted):
 *   T_imu_lidar : p_imu = R * p_lidar + t
 *   T_cam_imu   : p_cam = R * p_imu + t
 *   T_cam_lidar : p_cam = R * p_lidar + t  (optional; else = T_cam_imu * T_imu_lidar)
 *   T_base_wheel: p_base = R * p_wheel + t
 *
 * Camera optical undistortion → sensor/camera::* (use CameraIntrinsics as source).
 * Lidar motion undistortion → frontend/lio::ImuProcess + T_imu_lidar.
 */
struct CalibrationBundle {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CameraIntrinsics camera;
    CameraIntrinsics camera_right;  // stereo optional
    ImuIntrinsics imu;
    LidarIntrinsics lidar;

    ExtrinsicSE3 T_imu_lidar;
    ExtrinsicSE3 T_cam_imu;
    ExtrinsicSE3 T_cam_lidar;
    ExtrinsicSE3 T_base_wheel;
    bool has_T_cam_lidar = false;

    double time_offset_imu = 0.0;
    double time_offset_lidar = 0.0;
    double time_offset_cam = 0.0;
    double time_offset_wheel = 0.0;

    [[nodiscard]] Mat44_t T_cam_lidar_composed() const {
        if (has_T_cam_lidar) {
            return T_cam_lidar.T();
        }
        return T_cam_imu.T() * T_imu_lidar.T();
    }

    //! Fill legacy common::Extrinsics (lidar→camera + time offsets).
    [[nodiscard]] common::Extrinsics ToExtrinsics() const;

    //! Push T_imu_lidar (+ optional ESKF noise) into LocalEstimator.
    void ApplyTo(frontend::LocalEstimator* estimator) const;
};

CalibrationBundle LoadCalibrationBundle(const std::string& yaml_path);
CalibrationBundle LoadCalibrationBundle(const YAML::Node& root);
bool SaveCalibrationBundle(const CalibrationBundle& bundle,
                           const std::string& yaml_path);

}  // namespace calibration
}  // namespace autonomy::localization::atlas

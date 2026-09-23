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
 * @file constants.hpp
 * @brief Atlas SLAM Autolink topic / frame-name constants (for RViz / Autoviz).
 *
 * Shared by `SlamVisualizer` and subscribers; keep channel names in sync with
 * visualization configs when renaming.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_SYSTEM_CONSTANTS_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_SYSTEM_CONSTANTS_HPP_

namespace autonomy {
namespace localization {
namespace atlas {

// --- TF frames ---
constexpr char kMapFrame[] = "map";        ///< World / map frame
constexpr char kOdomFrame[] = "odom";      ///< Odometry frame (reserved)
constexpr char kBodyFrame[] = "body";      ///< IMU / body frame
constexpr char kCameraFrame[] = "camera";  ///< Camera frame

// --- Dynamic TF ---
/** @brief Dynamic TF for map→body (and map→camera), corresponding to `/tf`. */
constexpr char kTfChannel[] = "/tf";

// --- Trajectory / odometry ---
/** @brief Body / keyframe trajectory (nav_msgs/Path). */
constexpr char kTrajectoryChannel[] = "/autonomy/localization/atlas/trajectory";
/** @brief Live body odometry (nav_msgs/Odometry). */
constexpr char kOdometryChannel[] = "/autonomy/localization/atlas/odometry";

// --- Feature point clouds ---
/** @brief Local map points from current tracking (sensor_msgs/PointCloud2). */
constexpr char kLocalMapPointsChannel[] =
    "/autonomy/localization/atlas/local_map_points";
/** @brief Global sparse map points (sensor_msgs/PointCloud2). */
constexpr char kGlobalMapPointsChannel[] =
    "/autonomy/localization/atlas/global_map_points";

// --- Loop / camera markers ---
/** @brief Loop-closure edges MarkerArray (visualization_msgs/MarkerArray). */
constexpr char kLoopClosureChannel[] =
    "/autonomy/localization/atlas/loop_closure";
/** @brief Keyframe camera frustums (visualization_msgs/MarkerArray). */
constexpr char kCameraFrustumChannel[] =
    "/autonomy/localization/atlas/camera_frustums";
/** @brief Highlighted current-camera frustum (visualization_msgs/Marker). */
constexpr char kCurrentCameraChannel[] =
    "/autonomy/localization/atlas/current_camera";

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_SYSTEM_CONSTANTS_HPP_

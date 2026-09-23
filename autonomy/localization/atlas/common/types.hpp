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
 * @file types.hpp
 * @brief Shared Atlas scalars, poses, landmarks, and system-level keyframe /
 *        odometry types.
 *
 * Type foundation for localization/atlas: frontend, backend, and map modules
 * should prefer these aliases to avoid mixing float/double or SE3 flavors.
 * Pose conventions aligned with ORB-SLAM3 are documented per struct; system
 * `Keyframe` / `OdometryResult` target MapManager / visualization and differ
 * from in-map `map::KeyFrame`.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_COMMON_TYPES_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_COMMON_TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace autonomy {
namespace localization {
namespace atlas {

using Scalar = double;  ///< Default Atlas floating precision (Ceres / Eigen Isometry3d)

using Vec2 = Eigen::Matrix<Scalar, 2, 1>;  ///< 2D vector / pre-homogeneous pixel
using Vec3 = Eigen::Matrix<Scalar, 3, 1>;  ///< 3D point or translation
using Vec6 = Eigen::Matrix<Scalar, 6, 1>;  ///< 6D error or twist
using Mat33 = Eigen::Matrix<Scalar, 3, 3>;  ///< 3x3 rotation or covariance block
using Mat66 = Eigen::Matrix<Scalar, 6, 6>;  ///< 6x6 pose covariance, etc.
using MatXX = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;  ///< Dynamic matrix
using Quat = Eigen::Quaternion<Scalar>;  ///< Unit quaternion (Hamilton)

/**
 * @typedef SE3
 * @brief Rigid transform, Eigen::Isometry3d.
 *
 * Stored as a 4x4 homogeneous matrix; `.linear()` is rotation,
 * `.translation()` is translation. In Tracking / KeyFrame this is usually
 * **T_cw (camera ← world)**; in system `OdometryResult` it is **T_wb
 * (world ← body)**. Always check the call-site docs.
 */
using SE3 = Eigen::Isometry3d;

/**
 * @brief Return identity SE3 (R=I, t=0).
 * @return Identity rigid transform.
 */
inline SE3 SE3Identity() { return SE3::Identity(); }

/**
 * @brief Extract SE3 translation.
 * @param[in] pose Input pose.
 * @return Translation vector t.
 */
inline Vec3 SE3Translation(const SE3& pose) { return pose.translation(); }

/**
 * @brief Extract SE3 rotation as a quaternion.
 * @param[in] pose Input pose.
 * @return Quaternion corresponding to `pose.linear()`.
 */
inline Quat SE3Rotation(const SE3& pose) { return Quat(pose.rotation()); }

/**
 * @brief Extract SE3 rotation matrix.
 * @param[in] pose Input pose.
 * @return 3x3 rotation matrix R.
 */
inline Mat33 SE3RotationMatrix(const SE3& pose) { return pose.rotation(); }

/**
 * @brief Build SE3 from translation and quaternion.
 * @param[in] translation Translation t.
 * @param[in] rotation Rotation (normalized internally).
 * @return Composed rigid transform.
 */
inline SE3 MakeSE3(const Vec3& translation, const Quat& rotation) {
    SE3 pose = SE3::Identity();
    pose.linear() = rotation.normalized().toRotationMatrix();
    pose.translation() = translation;
    return pose;
}

/**
 * @brief Invert SE3 (T⁻¹).
 * @param[in] pose Input pose.
 * @return pose.inverse().
 */
inline SE3 SE3Inverse(const SE3& pose) { return pose.inverse(); }

using TimeStamp = int64_t;  ///< Timestamp (prefer nanoseconds); match sensor bags

inline constexpr TimeStamp kInvalidTime = -1;  ///< Invalid / unset timestamp

/**
 * @enum autonomy::localization::atlas::FrontendMode
 * @brief Frontend operating mode (config `mode` field).
 */
enum class FrontendMode {
    kVo = 0,   ///< Pure visual odometry
    kVio,      ///< Visual-inertial odometry
    kLo,       ///< LiDAR odometry (reserved)
    kLio,      ///< LiDAR-inertial (reserved)
    kLivo,     ///< LiDAR-inertial-visual (reserved)
};

/**
 * @enum autonomy::localization::atlas::BackendType
 * @brief Backend optimizer selection (config).
 */
enum class BackendType {
    kIekf = 0,  ///< Iterated EKF (reserved)
    kGraph,     ///< Pose graph (reserved)
    kCeres,     ///< Ceres nonlinear least squares (primary path)
};

/**
 * @enum autonomy::localization::atlas::FusionStyle
 * @brief Loose vs tight multi-sensor fusion style.
 */
enum class FusionStyle {
    kLoose = 0,  ///< Loose coupling
    kTight,      ///< Tight coupling (primary VIO path)
};

/**
 * @enum autonomy::localization::atlas::SlamState
 * @brief System-level SLAM state machine (external; may map from Tracker::State).
 */
enum class SlamState {
    kUninitialized = 0,  ///< No image received yet
    kInitializing,       ///< Initializing (mono two-view / stereo mapping)
    kTracking,           ///< Normal tracking
    kLost,               ///< Tracking lost
    kDegraded,           ///< Degraded (e.g. IMU-only briefly)
};

/**
 * @brief Convert frontend mode to a config string.
 * @param[in] mode Enum value.
 * @return `"vo"` / `"vio"` / `"lo"` / `"lio"` / `"livo"`; `"unknown"` if unrecognized.
 */
inline std::string ToString(FrontendMode mode) {
    switch (mode) {
        case FrontendMode::kVo:
            return "vo";
        case FrontendMode::kVio:
            return "vio";
        case FrontendMode::kLo:
            return "lo";
        case FrontendMode::kLio:
            return "lio";
        case FrontendMode::kLivo:
            return "livo";
    }
    return "unknown";
}

/**
 * @brief Parse a config string into a frontend mode.
 * @param[in] mode Lowercase string (e.g. `"vio"`).
 * @return Matching enum; defaults to `kVio` if unrecognized.
 */
inline FrontendMode ParseFrontendMode(const std::string& mode) {
    if (mode == "vo") {
        return FrontendMode::kVo;
    }
    if (mode == "lo") {
        return FrontendMode::kLo;
    }
    if (mode == "lio") {
        return FrontendMode::kLio;
    }
    if (mode == "livo") {
        return FrontendMode::kLivo;
    }
    return FrontendMode::kVio;
}

/**
 * @struct autonomy::localization::atlas::Landmark
 * @brief Lightweight landmark (system / visualization; not map `MapPoint`).
 */
struct Landmark {
    int id = -1;                      ///< Landmark ID; -1 if unset
    Vec3 position = Vec3::Zero();     ///< 3D position in world frame
    Vec2 uv = Vec2::Zero();           ///< Optional pixel observation
    bool has_uv = false;              ///< Whether `uv` is valid
    Scalar inverse_depth = 0.0;       ///< Inverse-depth parameterization (reserved)
};

/**
 * @struct autonomy::localization::atlas::PointXYZI
 * @brief LiDAR point with intensity and timestamp.
 */
struct PointXYZI {
    float x = 0.f;           ///< X [m]
    float y = 0.f;           ///< Y [m]
    float z = 0.f;           ///< Z [m]
    float intensity = 0.f;   ///< Reflectance intensity
    double timestamp = 0.0;  ///< Point time (sec or relative to scan; frontend convention)
};

using PointCloud = std::vector<PointXYZI>;  ///< Point-cloud alias

/**
 * @struct autonomy::localization::atlas::Covariance6
 * @brief Wrapper for a 6x6 pose covariance.
 */
struct Covariance6 {
    Mat66 matrix = Mat66::Identity();  ///< Covariance matrix (default identity)
};

/**
 * @struct autonomy::localization::atlas::Keyframe
 * @brief System-level keyframe summary (frontend → MapManager / visualization).
 *
 * Unlike `map::KeyFrame`, this does not hold ORB descriptors, covisibility,
 * etc.; only pose and optional landmark / cloud snapshots.
 *
 * @note `pose_world_body` is **T_wb** (world ← body/IMU).
 */
struct Keyframe {
    int id = -1;                              ///< Keyframe index
    TimeStamp timestamp = kInvalidTime;       ///< Timestamp
    SE3 pose_world_body = SE3Identity();      ///< T_wb
    std::vector<Landmark> landmarks;          ///< Associated landmark snapshot
    PointCloud cloud;                         ///< Optional local cloud
};

/**
 * @struct autonomy::localization::atlas::OdometryResult
 * @brief Per-frame tracking output (pose, velocity, biases, covariance, validity).
 *
 * Filled by `Tracker` in each `PublishResult` for navigation or trajectory logging.
 *
 * @note `pose_world_body` is **T_wb**; convert via calibration when internal
 *       vision uses T_cw.
 */
struct OdometryResult {
    TimeStamp timestamp = kInvalidTime;   ///< Time of this result
    SE3 pose_world_body = SE3Identity();  ///< T_wb
    Vec3 velocity = Vec3::Zero();         ///< World-frame linear velocity [m/s] (VIO)
    Vec3 gyro_bias = Vec3::Zero();        ///< Gyro bias
    Vec3 accel_bias = Vec3::Zero();       ///< Accel bias
    Covariance6 covariance;               ///< Pose covariance (optional)
    std::vector<Landmark> landmarks;      ///< Landmarks visible in this frame
    PointCloud local_map;                 ///< Local map cloud (visualization)
    bool valid = false;                   ///< Whether this frame result is trustworthy
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_COMMON_TYPES_HPP_

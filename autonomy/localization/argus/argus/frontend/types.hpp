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
 * @brief Front-end measure / state types (FAST-LIVO2 MeasureGroup /
 * StatesGroup).
 */

#ifndef ARGUS_FRONTEND_TYPES_HPP_
#define ARGUS_FRONTEND_TYPES_HPP_

#include <deque>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "argus/common/math/so3.hpp"
#include "argus/sensor/imu_data.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace frontend {

constexpr double kGravity = 9.81;
constexpr int kDimState = 19;
constexpr double kInitCov = 0.01;

#define ARGUS_VEC_FROM_ARRAY(v) (v)[0], (v)[1], (v)[2]
#define ARGUS_MAT_FROM_ARRAY(v) \
    (v)[0], (v)[1], (v)[2], (v)[3], (v)[4], (v)[5], (v)[6], (v)[7], (v)[8]

#ifndef VEC_FROM_ARRAY
#define VEC_FROM_ARRAY(v) ARGUS_VEC_FROM_ARRAY(v)
#endif
#ifndef MAT_FROM_ARRAY
#define MAT_FROM_ARRAY(v) ARGUS_MAT_FROM_ARRAY(v)
#endif

#ifndef MD
#define MD(a, b) Eigen::Matrix<double, (a), (b)>
#endif
#ifndef VD
#define VD(a) Eigen::Matrix<double, (a), 1>
#endif

/** @brief LiDAR / IMU / camera fusion mode for SensorSync and ESIKF. */
enum SlamMode {
    kOnlyLo = 0,
    kOnlyLio = 1,
    kLivo = 2,
    /** @brief IMU + camera only (no LiDAR); VIO packages from image timestamps.
     */
    kOnlyVio = 3,
    // Legacy aliases (FAST-LIVO2 / LIVO).
    ONLY_LO = kOnlyLo,
    ONLY_LIO = kOnlyLio,
    LIVO = kLivo,
    ONLY_VIO = kOnlyVio
};

/** @brief ESIKF update phase within a LiDAR measure group. */
enum EkfState {
    kWait = 0,
    kVio = 1,
    kLio = 2,
    kLo = 3,
    WAIT = kWait,
    VIO = kVio,
    LIO = kLio,
    LO = kLo
};

/** @brief IMU pose stamp along a scan (relative time + kinematics). */
struct Pose6D {
    double offset_time = 0;
    double acc[3] = {0, 0, 0};
    double gyr[3] = {0, 0, 0};
    double vel[3] = {0, 0, 0};
    double pos[3] = {0, 0, 0};
    double rot[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
};

/** @brief One VIO/LIO sub-window: IMU deque + optional image. */
struct MeasureGroup {
    double vio_time = 0.0;
    double lio_time = 0.0;
    std::deque<sensor::ImuData> imu;
    cv::Mat image;
};

/** @brief Full LiDAR scan bundle with nested measure groups and EKF flag. */
struct LidarMeasureGroup {
    double lidar_frame_beg_time = 0.0;
    double lidar_frame_end_time = 0.0;
    double last_lio_update_time = -1.0;
    sensor::PointCloudXYZI::Ptr lidar;
    sensor::PointCloudXYZI::Ptr cloud_proc_cur;
    sensor::PointCloudXYZI::Ptr cloud_proc_next;
    std::deque<MeasureGroup> measures;
    EkfState ekf_phase = kWait;
    int lidar_scan_index_now = 0;

    LidarMeasureGroup() {
        lidar = sensor::PointCloudXYZI::make_shared();
        cloud_proc_cur = sensor::PointCloudXYZI::make_shared();
        cloud_proc_next = sensor::PointCloudXYZI::make_shared();
    }
};

/** @brief 19-D ESIKF state (pose, biases, gravity, exposure) and covariance. */
struct StatesGroup {
    StatesGroup() {
        rot_end = Eigen::Matrix3d::Identity();
        pos_end = Eigen::Vector3d::Zero();
        vel_end = Eigen::Vector3d::Zero();
        bias_g = Eigen::Vector3d::Zero();
        bias_a = Eigen::Vector3d::Zero();
        gravity = Eigen::Vector3d::Zero();
        inv_expo_time = 1.0;
        cov = MD(kDimState, kDimState)::Identity() * kInitCov;
        cov(6, 6) = 0.00001;
        cov.block<9, 9>(10, 10) = MD(9, 9)::Identity() * 0.00001;
    }

    StatesGroup(const StatesGroup& b) = default;
    StatesGroup& operator=(const StatesGroup& b) = default;

    /**
     * @brief Add a tangent-space increment on the left of pose and Euclidean
     * blocks.
     * @param state_add 19-D increment (rotation log at 0..2).
     * @return Updated state copy; covariance unchanged.
     */
    StatesGroup operator+(
        const Eigen::Matrix<double, kDimState, 1>& state_add) const {
        StatesGroup a;
        a.rot_end =
            rot_end * math::Exp(state_add(0), state_add(1), state_add(2));
        a.pos_end = pos_end + state_add.block<3, 1>(3, 0);
        a.inv_expo_time = inv_expo_time + state_add(6, 0);
        a.vel_end = vel_end + state_add.block<3, 1>(7, 0);
        a.bias_g = bias_g + state_add.block<3, 1>(10, 0);
        a.bias_a = bias_a + state_add.block<3, 1>(13, 0);
        a.gravity = gravity + state_add.block<3, 1>(16, 0);
        a.cov = cov;
        return a;
    }

    /**
     * @brief In-place tangent-space increment (same layout as operator+).
     * @param state_add 19-D increment.
     * @return Reference to this.
     */
    StatesGroup& operator+=(
        const Eigen::Matrix<double, kDimState, 1>& state_add) {
        rot_end = rot_end * math::Exp(state_add(0), state_add(1), state_add(2));
        pos_end += state_add.block<3, 1>(3, 0);
        inv_expo_time += state_add(6, 0);
        vel_end += state_add.block<3, 1>(7, 0);
        bias_g += state_add.block<3, 1>(10, 0);
        bias_a += state_add.block<3, 1>(13, 0);
        gravity += state_add.block<3, 1>(16, 0);
        return *this;
    }

    /**
     * @brief Tangent difference `this - b` in the ESIKF error state.
     * @param b Reference state.
     * @return 19-D error vector (rotation log relative to `b`).
     */
    Eigen::Matrix<double, kDimState, 1> operator-(const StatesGroup& b) const {
        Eigen::Matrix<double, kDimState, 1> a;
        const Eigen::Matrix3d rotd = b.rot_end.transpose() * rot_end;
        a.block<3, 1>(0, 0) = math::Log(rotd);
        a.block<3, 1>(3, 0) = pos_end - b.pos_end;
        a(6, 0) = inv_expo_time - b.inv_expo_time;
        a.block<3, 1>(7, 0) = vel_end - b.vel_end;
        a.block<3, 1>(10, 0) = bias_g - b.bias_g;
        a.block<3, 1>(13, 0) = bias_a - b.bias_a;
        a.block<3, 1>(16, 0) = gravity - b.gravity;
        return a;
    }

    /** @brief Reset rotation, position, and velocity to identity / zero. */
    void ResetPose() {
        rot_end = Eigen::Matrix3d::Identity();
        pos_end = Eigen::Vector3d::Zero();
        vel_end = Eigen::Vector3d::Zero();
    }

    Eigen::Matrix3d rot_end = Eigen::Matrix3d::Identity();
    Eigen::Vector3d pos_end = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_end = Eigen::Vector3d::Zero();
    double inv_expo_time = 1.0;
    Eigen::Vector3d bias_g = Eigen::Vector3d::Zero();
    Eigen::Vector3d bias_a = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, kDimState, kDimState> cov =
        Eigen::Matrix<double, kDimState, kDimState>::Identity() * kInitCov;
};

/**
 * @brief Pack IMU kinematics and pose into a legacy Pose6D stamp.
 * @param t Relative time offset (s).
 * @param a Linear acceleration (m/s²).
 * @param g Angular rate (rad/s).
 * @param v Velocity (m/s).
 * @param p Position (m).
 * @param R Rotation matrix body→reference.
 * @return Filled Pose6D.
 */
template <typename T>
Pose6D SetPose6D(const double t, const Eigen::Matrix<T, 3, 1>& a,
                 const Eigen::Matrix<T, 3, 1>& g,
                 const Eigen::Matrix<T, 3, 1>& v,
                 const Eigen::Matrix<T, 3, 1>& p,
                 const Eigen::Matrix<T, 3, 3>& R) {
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; ++i) {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; ++j) {
            rot_kp.rot[i * 3 + j] = R(i, j);
        }
    }
    return rot_kp;
}

/** @brief Legacy alias for SetPose6D (same parameters and return). */
template <typename T>
auto set_pose6d(const double t, const Eigen::Matrix<T, 3, 1>& a,
                const Eigen::Matrix<T, 3, 1>& g,
                const Eigen::Matrix<T, 3, 1>& v,
                const Eigen::Matrix<T, 3, 1>& p,
                const Eigen::Matrix<T, 3, 3>& R) {
    return SetPose6D(t, a, g, v, p, R);
}

/** @brief LiDAR point with body/world coords and covariance (LIO / VIO). */
struct PointWithVar {
    Eigen::Vector3d point_b = Eigen::Vector3d::Zero();
    Eigen::Vector3d point_i = Eigen::Vector3d::Zero();
    Eigen::Vector3d point_w = Eigen::Vector3d::Zero();
    Eigen::Matrix3d var_nostate = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d body_var = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d var = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d point_crossmat = Eigen::Matrix3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
};

using pointWithVar = PointWithVar;

}  // namespace frontend
}  // namespace argus

#endif  // ARGUS_FRONTEND_TYPES_HPP_

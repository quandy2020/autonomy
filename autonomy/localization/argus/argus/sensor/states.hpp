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
 * @file states.hpp
 * @brief Front-end measure / state types for Voxel-SLAM LIO.
 */

#ifndef ARGUS_FRONTEND_TYPES_HPP_
#define ARGUS_FRONTEND_TYPES_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <deque>
#include <memory>

#include "common/math/so3.hpp"
#include "sensor/imu_types.hpp"
#include "sensor/point_types.hpp"

namespace argus {
namespace vxslam {
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

/**
 * @brief LiDAR / IMU packaging mode for @ref SensorSync.
 */
enum SlamMode {
  kOnlyLo = 0,
  kOnlyLio = 1,
  ONLY_LO = kOnlyLo,
  ONLY_LIO = kOnlyLio,
};

/**
 * @brief Update phase within a LiDAR measure group.
 */
enum EkfState {
  kWait = 0,
  kLio = 2,
  kLo = 3,
  WAIT = kWait,
  LIO = kLio,
  LO = kLo
};

/**
 * @brief IMU pose stamp along a scan (relative time + kinematics).
 * @var offset_time Relative time from scan begin (seconds).
 * @var acc         Linear acceleration (m/s²), length 3.
 * @var gyr         Angular rate (rad/s), length 3.
 * @var vel         Velocity (m/s), length 3.
 * @var pos         Position (m), length 3.
 * @var rot         Row-major 3×3 rotation matrix (body → reference).
 */
struct Pose6D {
  double offset_time = 0;
  double acc[3] = {0, 0, 0};
  double gyr[3] = {0, 0, 0};
  double vel[3] = {0, 0, 0};
  double pos[3] = {0, 0, 0};
  double rot[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
};

/**
 * @brief One LIO sub-window: IMU deque spanning the scan.
 * @var lio_time LIO update time (seconds).
 * @var imu      IMU samples spanning the window.
 */
struct MeasureGroup {
  double lio_time = 0.0;
  std::deque<sensor::ImuData> imu;
};

/**
 * @brief Full LiDAR scan bundle with nested measure groups and EKF phase.
 * @var lidar_frame_beg_time Sweep begin time (seconds).
 * @var lidar_frame_end_time Sweep end time (seconds).
 * @var last_lio_update_time Last LIO update time; -1 if none.
 * @var lidar                Raw / deskewed sweep cloud.
 * @var cloud_proc_cur       Current processed cloud buffer.
 * @var cloud_proc_next      Next processed cloud buffer.
 * @var measures             Nested VIO/LIO measure windows.
 * @var ekf_phase            Current @ref EkfState.
 * @var lidar_scan_index_now Active scan index within the group.
 */
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

/**
 * @brief 19-D ESIKF state (pose, biases, gravity, exposure) and covariance.
 * Layout: rot(0..2), pos(3..5), inv_expo(6), vel(7..9), bias_g(10..12),
 * bias_a(13..15), gravity(16..18).
 * @var rot_end      Rotation at scan end (body → world).
 * @var pos_end      Position at scan end (metres).
 * @var vel_end      Velocity at scan end (m/s).
 * @var inv_expo_time Inverse exposure time (LIVO photometric).
 * @var bias_g       Gyro bias (rad/s).
 * @var bias_a       Accel bias (m/s²).
 * @var gravity      Gravity vector in world frame (m/s²).
 * @var cov          @c kDimState × @c kDimState state covariance.
 */
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
   * @brief Add a tangent-space increment (left of pose, Euclidean blocks).
   * @param[in] state_add 19-D increment; rotation log at indices 0..2.
   * @return Updated state copy; covariance copied unchanged.
   */
  StatesGroup operator+(
      const Eigen::Matrix<double, kDimState, 1>& state_add) const {
    StatesGroup a;
    a.rot_end = rot_end * math::Exp(state_add(0), state_add(1), state_add(2));
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
   * @brief In-place tangent-space increment (same layout as @c operator+).
   * @param[in] state_add 19-D increment; rotation log at indices 0..2.
   * @return Reference to this state.
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
   * @brief Tangent difference @c this - b in the ESIKF error state.
   * @param[in] b Reference state.
   * @return 19-D error vector; rotation is @c Log(b.Rᵀ R).
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

  /**
   * @brief Reset rotation, position, and velocity to identity / zero.
   */
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
 * @brief Pack IMU kinematics and pose into a legacy @ref Pose6D stamp.
 * @tparam T Scalar type of the Eigen inputs (typically @c double / @c float).
 * @param[in] t Relative time offset (seconds).
 * @param[in] a Linear acceleration (m/s²).
 * @param[in] g Angular rate (rad/s).
 * @param[in] v Velocity (m/s).
 * @param[in] p Position (metres).
 * @param[in] R Rotation matrix body → reference.
 * @return Filled @ref Pose6D with row-major @c rot.
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

/**
 * @brief Legacy alias for @ref SetPose6D (same parameters and return).
 * @tparam T Scalar type of the Eigen inputs.
 * @param[in] t Relative time offset (seconds).
 * @param[in] a Linear acceleration (m/s²).
 * @param[in] g Angular rate (rad/s).
 * @param[in] v Velocity (m/s).
 * @param[in] p Position (metres).
 * @param[in] R Rotation matrix body → reference.
 * @return Result of @ref SetPose6D.
 */
template <typename T>
auto set_pose6d(const double t, const Eigen::Matrix<T, 3, 1>& a,
                const Eigen::Matrix<T, 3, 1>& g,
                const Eigen::Matrix<T, 3, 1>& v,
                const Eigen::Matrix<T, 3, 1>& p,
                const Eigen::Matrix<T, 3, 3>& R) {
  return SetPose6D(t, a, g, v, p, R);
}

/**
 * @brief LiDAR point with body/world coords and covariance (LIO / VIO).
 * @var point_b       Point in body / LiDAR frame.
 * @var point_i       Intermediate / IMU-frame point.
 * @var point_w       Point in world frame.
 * @var var_nostate   Covariance without state uncertainty.
 * @var body_var      Body-frame measurement covariance.
 * @var var           Full propagated covariance.
 * @var point_crossmat Skew-symmetric matrix of the point.
 * @var normal        Surface normal (unitless).
 */
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
}  // namespace vxslam
}  // namespace argus

#endif  // ARGUS_FRONTEND_TYPES_HPP_

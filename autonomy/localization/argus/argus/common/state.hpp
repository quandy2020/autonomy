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
 * @file state.hpp
 * @brief Navigation state and pose type aliases shared by estimators.
 */

#ifndef ARGUS_COMMON_STATE_HPP_
#define ARGUS_COMMON_STATE_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "argus/common/rigid_transform.hpp"

namespace argus {
namespace common {

/**
 * @brief Rigid body pose in SE(3).
 * Alias of @ref transform::Rigid3d (translation + unit quaternion).
 */
using Pose = transform::Rigid3d;

/**
 * @brief 6×6 pose covariance on the SE(3) tangent space.
 * Layout convention: rotation block first, then translation (callers must
 * document any deviation).
 */
using PoseCovariance = Eigen::Matrix<double, 6, 6>;

/**
 * @brief Generic navigation state for IMU-driven filters.
 * Holds pose, velocity, inertial biases, and gravity. Must not embed sensor
 * payloads such as point clouds or images.
 * @var pose       Body pose in the world / map frame.
 * @var velocity   Linear velocity in the world frame (m/s).
 * @var gyro_bias  Gyroscope bias (rad/s).
 * @var accel_bias Accelerometer bias (m/s²).
 * @var gravity    Gravity vector in the world frame (m/s²); default ≈ −gẑ.
 */
struct State {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Pose pose = Pose::Identity();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, -9.81);
};

/**
 * @brief Dense covariance companion for @ref State.
 * Exact block layout is defined by the owning filter (for example 15×15 ESIKF).
 */
using Covariance = Eigen::MatrixXd;

}  // namespace common

/** @brief Temporary alias of @ref common during migration (`core::State`). */
namespace core = common;

}  // namespace argus

#endif  // ARGUS_COMMON_STATE_HPP_

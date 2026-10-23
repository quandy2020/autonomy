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
 * @brief Lidar-inertial navigation state for the voxel frontend.
 */

#ifndef ATLAS_FRONTEND_LIDAR_STATE_HPP_
#define ATLAS_FRONTEND_LIDAR_STATE_HPP_

#include <Eigen/Core>

#include "common/transform/rigid_transform.hpp"

namespace atlas {
namespace frontend {
namespace lidar {

inline Eigen::Matrix<double, 15, 15> DefaultStateCovariance() {
  Eigen::Matrix<double, 15, 15> covariance =
      Eigen::Matrix<double, 15, 15>::Identity() * 1e-4;
  covariance.block<6, 6>(9, 9).setIdentity();
  covariance.block<6, 6>(9, 9) *= 1e-5;
  return covariance;
}

struct State {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double time = 0.0;
  transform::Rigid3d pose = transform::Rigid3d::Identity();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, -9.81);
  Eigen::Matrix<double, 15, 15> covariance = DefaultStateCovariance();
};

}  // namespace lidar
}  // namespace frontend
}  // namespace atlas

#endif  // ATLAS_FRONTEND_LIDAR_STATE_HPP_

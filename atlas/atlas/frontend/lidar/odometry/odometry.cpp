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

#include "frontend/lidar/odometry/odometry.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include "common/math/so3.hpp"

namespace atlas {
namespace frontend {
namespace lidar {

Odometry::PredictResult Odometry::Predict(
    const State& state, const std::vector<sensor::ImuData>& imu) const {
  PredictResult result;
  result.state = state;
  double time = state.time;
  Eigen::Matrix3d rotation = state.pose.rotation().toRotationMatrix();
  Eigen::Vector3d position = state.pose.translation();
  Eigen::Vector3d velocity = state.velocity;
  Eigen::Matrix<double, 15, 15> covariance = state.covariance;
  bool stepped = false;
  for (const sensor::ImuData& sample : imu) {
    if (sample.time <= time) {
      continue;
    }
    const double dt = sample.time - time;
    const Eigen::Vector3d omega = sample.angular_velocity - state.gyro_bias;
    const Eigen::Vector3d specific =
        sample.linear_acceleration - state.accel_bias;
    Eigen::Matrix<double, 15, 15> transition =
        Eigen::Matrix<double, 15, 15>::Identity();
    Eigen::Matrix<double, 15, 15> noise = Eigen::Matrix<double, 15, 15>::Zero();
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    transition.block<3, 3>(0, 0) = math::Exp(omega, -dt);
    transition.block<3, 3>(0, 9) = -identity * dt;
    transition.block<3, 3>(3, 6) = identity * dt;
    transition.block<3, 3>(6, 0) =
        -rotation * math::SkewSymmetric(specific) * dt;
    transition.block<3, 3>(6, 12) = -rotation * dt;
    const double dt2 = dt * dt;
    noise.block<3, 3>(0, 0) = identity * (config_.gyro_noise * dt2);
    noise.block<3, 3>(6, 6) = rotation * (identity * config_.accel_noise) *
                              rotation.transpose() * dt2;
    noise.block<3, 3>(9, 9) = identity * (config_.gyro_bias_noise * dt2);
    noise.block<3, 3>(12, 12) = identity * (config_.accel_bias_noise * dt2);
    covariance = transition * covariance * transition.transpose() + noise;
    rotation = rotation * math::Exp(omega, dt);
    const Eigen::Vector3d acceleration = rotation * specific + state.gravity;
    position += velocity * dt + 0.5 * acceleration * dt * dt;
    velocity += acceleration * dt;
    time = sample.time;
    stepped = true;
  }
  if (!stepped) {
    return result;
  }
  result.state.time = time;
  result.state.pose =
      transform::Rigid3d(position, Eigen::Quaterniond(rotation));
  result.state.velocity = velocity;
  result.state.covariance = 0.5 * (covariance + covariance.transpose());
  result.propagated = true;
  return result;
}

Odometry::UpdateResult Odometry::Update(const State& predicted,
                                        const sensor::TimedPointCloud& cloud,
                                        const PlaneMap& map) {
  UpdateResult result;
  result.state = predicted;
  const Eigen::Matrix3d predicted_rotation =
      predicted.pose.rotation().toRotationMatrix();
  Eigen::Matrix3d rotation = predicted_rotation;
  Eigen::Vector3d position = predicted.pose.translation();
  Eigen::Vector3d velocity = predicted.velocity;
  Eigen::Vector3d gyro_bias = predicted.gyro_bias;
  Eigen::Vector3d accel_bias = predicted.accel_bias;
  Eigen::Matrix<double, 15, 15> covariance = predicted.covariance;
  covariance.diagonal().array() += 1e-12;
  const Eigen::Matrix<double, 15, 15> covariance_inverse = covariance.inverse();
  Eigen::Matrix3d normal_scatter = Eigen::Matrix3d::Zero();
  Eigen::Matrix<double, 15, 15> gain = Eigen::Matrix<double, 15, 15>::Zero();
  int matches = 0;
  bool updated = false;

  for (int iteration = 0; iteration < config_.max_iterations; ++iteration) {
    Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> gradient = Eigen::Matrix<double, 6, 1>::Zero();
    normal_scatter.setZero();
    matches = 0;
    for (const sensor::TimedRangefinderPoint& point : cloud) {
      const Eigen::Vector3d body = point.position.cast<double>();
      const Eigen::Vector3d world = rotation * body + position;
      Plane plane;
      if (!map.Query(world, &plane)) {
        continue;
      }
      const double residual = plane.normal.dot(world - plane.center);
      Eigen::Matrix<double, 6, 1> jacobian;
      jacobian.head<3>() =
          math::SkewSymmetric(body) * rotation.transpose() * plane.normal;
      jacobian.tail<3>() = plane.normal;
      hessian += jacobian * jacobian.transpose();
      gradient -= jacobian * residual;
      normal_scatter += plane.normal * plane.normal.transpose();
      ++matches;
    }
    if (matches < 3) {
      break;
    }
    Eigen::Matrix<double, 15, 15> information = covariance_inverse;
    information.topLeftCorner<6, 6>() += hessian;
    const Eigen::Matrix<double, 15, 15> kalman = information.inverse();
    gain.setZero();
    gain.leftCols(6) = kalman.leftCols(6) * hessian;
    Eigen::Matrix<double, 15, 1> error;
    const Eigen::Matrix3d rotation_error =
        rotation.transpose() * predicted_rotation;
    error.segment<3>(0) = math::Log(rotation_error);
    error.segment<3>(3) = predicted.pose.translation() - position;
    error.segment<3>(6) = predicted.velocity - velocity;
    error.segment<3>(9) = predicted.gyro_bias - gyro_bias;
    error.segment<3>(12) = predicted.accel_bias - accel_bias;
    const Eigen::Matrix<double, 15, 1> solution =
        kalman.leftCols(6) * gradient + error -
        gain.leftCols(6) * error.head<6>();
    rotation = rotation * math::Exp(solution.head<3>());
    position += solution.segment<3>(3);
    velocity += solution.segment<3>(6);
    gyro_bias += solution.segment<3>(9);
    accel_bias += solution.segment<3>(12);
    updated = true;
    if (solution.head<3>().norm() * 57.3 < 0.01 &&
        solution.segment<3>(3).norm() * 100.0 < 0.015) {
      break;
    }
  }

  result.match_count = matches;
  result.state.pose =
      transform::Rigid3d(position, Eigen::Quaterniond(rotation));
  result.state.velocity = velocity;
  result.state.gyro_bias = gyro_bias;
  result.state.accel_bias = accel_bias;
  if (updated) {
    const Eigen::Matrix<double, 15, 15> posterior =
        (Eigen::Matrix<double, 15, 15>::Identity() - gain) * covariance;
    result.state.covariance = 0.5 * (posterior + posterior.transpose());
  }
  bool aligned = false;
  if (matches >= 3) {
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(normal_scatter);
    aligned = solver.eigenvalues()(0) >= config_.min_normal_eigenvalue;
  }
  if (aligned) {
    if (degrade_count_ > 0) {
      --degrade_count_;
    }
  } else {
    ++degrade_count_;
  }
  result.aligned = aligned;
  result.diverged = degrade_count_ > config_.degrade_bound;
  if (result.diverged) {
    degrade_count_ = 0;
  }
  return result;
}

}  // namespace lidar
}  // namespace frontend
}  // namespace atlas

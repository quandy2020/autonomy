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

#include "frontend/lidar/initialization/initializer.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <cmath>

#include "common/math/so3.hpp"

namespace argus {
namespace frontend {
namespace lidar {
namespace {

struct ImuDelta {
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  double dt = 0.0;
};

bool Integrate(const std::vector<sensor::ImuData>& imu, double scale,
               ImuDelta* delta) {
  if (delta == nullptr || imu.size() < 2) {
    return false;
  }
  *delta = ImuDelta{};
  for (std::size_t index = 1; index < imu.size(); ++index) {
    const double dt = imu[index].time - imu[index - 1].time;
    if (dt <= 0.0) {
      return false;
    }
    const Eigen::Vector3d gyro =
        0.5 * (imu[index].angular_velocity + imu[index - 1].angular_velocity);
    const Eigen::Vector3d acceleration =
        0.5 *
        (imu[index].linear_acceleration + imu[index - 1].linear_acceleration) *
        scale;
    const Eigen::Matrix3d rotation = delta->rotation;
    delta->position +=
        delta->velocity * dt + 0.5 * rotation * acceleration * dt * dt;
    delta->velocity += rotation * acceleration * dt;
    delta->rotation = rotation * math::Exp(gyro, dt);
    delta->dt += dt;
  }
  return delta->dt > 0.0;
}

}  // namespace

bool Initializer::TryInitialize(const std::vector<sensor::ImuData>& imu,
                                State* state) const {
  if (state == nullptr ||
      static_cast<int>(imu.size()) < config_.min_imu_samples) {
    return false;
  }
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const sensor::ImuData& sample : imu) {
    mean += sample.linear_acceleration;
  }
  mean /= static_cast<double>(imu.size());
  double variance = 0.0;
  for (const sensor::ImuData& sample : imu) {
    variance += (sample.linear_acceleration - mean).squaredNorm();
  }
  variance /= static_cast<double>(imu.size());
  if (variance > config_.max_accel_variance || mean.norm() < 1.0) {
    return false;
  }

  const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
      mean.normalized(), Eigen::Vector3d::UnitZ());
  state->time = imu.back().time;
  state->pose = transform::Rigid3d(Eigen::Vector3d::Zero(), rotation);
  state->velocity.setZero();
  state->gyro_bias.setZero();
  state->accel_bias.setZero();
  state->gravity = Eigen::Vector3d(0.0, 0.0, -config_.gravity_magnitude);
  return true;
}

bool Initializer::TryInitializeMotion(const std::vector<MotionFrame>& frames,
                                      std::vector<State>* states) const {
  if (states == nullptr ||
      static_cast<int>(frames.size()) < config_.min_motion_frames) {
    return false;
  }
  Eigen::Vector3d mean_acceleration = Eigen::Vector3d::Zero();
  int sample_count = 0;
  for (std::size_t index = 0; index + 1 < frames.size(); ++index) {
    for (const sensor::ImuData& sample : frames[index].imu_until_next) {
      mean_acceleration += sample.linear_acceleration;
      ++sample_count;
    }
  }
  if (sample_count == 0) {
    return false;
  }
  mean_acceleration /= static_cast<double>(sample_count);
  const double mean_norm = mean_acceleration.norm();
  const double scale = (mean_norm > 1e-3 && mean_norm < 2.0)
                           ? config_.gravity_magnitude / mean_norm
                           : 1.0;

  Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> gradient = Eigen::Matrix<double, 6, 1>::Zero();
  std::vector<ImuDelta> deltas;
  deltas.reserve(frames.size() - 1);
  double elapsed = 0.0;
  Eigen::Vector3d carried_velocity = Eigen::Vector3d::Zero();
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  for (std::size_t index = 0; index + 1 < frames.size(); ++index) {
    ImuDelta delta;
    if (!Integrate(frames[index].imu_until_next, scale, &delta)) {
      return false;
    }
    const Eigen::Matrix3d rotation =
        frames[index].pose.rotation().toRotationMatrix();
    const Eigen::Vector3d position_delta =
        frames[index + 1].pose.translation() - frames[index].pose.translation();
    const Eigen::Vector3d residual = position_delta -
                                     rotation * delta.position -
                                     delta.dt * carried_velocity;
    const double gravity_coeff = elapsed * delta.dt + 0.5 * delta.dt * delta.dt;
    hessian.block<3, 3>(0, 0) += delta.dt * delta.dt * identity;
    hessian.block<3, 3>(0, 3) += delta.dt * gravity_coeff * identity;
    hessian.block<3, 3>(3, 0) += delta.dt * gravity_coeff * identity;
    hessian.block<3, 3>(3, 3) += gravity_coeff * gravity_coeff * identity;
    gradient.head<3>() += delta.dt * residual;
    gradient.tail<3>() += gravity_coeff * residual;
    carried_velocity += rotation * delta.velocity;
    elapsed += delta.dt;
    deltas.push_back(delta);
  }

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigenvalues(
      hessian);
  if (eigenvalues.eigenvalues()[0] <= 0.0 ||
      eigenvalues.eigenvalues()[5] > 1e8 * eigenvalues.eigenvalues()[0]) {
    return false;
  }
  const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> decomposition(hessian);
  if (decomposition.info() != Eigen::Success) {
    return false;
  }
  const Eigen::Matrix<double, 6, 1> solution = decomposition.solve(gradient);
  const Eigen::Vector3d initial_velocity = solution.head<3>();
  const Eigen::Vector3d gravity = solution.tail<3>();
  if (std::abs(gravity.norm() - config_.gravity_magnitude) >
      config_.gravity_tolerance) {
    return false;
  }

  states->assign(frames.size(), State{});
  Eigen::Vector3d velocity = initial_velocity;
  elapsed = 0.0;
  carried_velocity.setZero();
  for (std::size_t index = 0; index < frames.size(); ++index) {
    State& state = (*states)[index];
    state.time = frames[index].time;
    state.pose = frames[index].pose;
    state.velocity = velocity;
    state.gyro_bias.setZero();
    state.accel_bias.setZero();
    state.gravity = gravity;
    if (index + 1 == frames.size()) {
      break;
    }
    elapsed += deltas[index].dt;
    carried_velocity += frames[index].pose.rotation() * deltas[index].velocity;
    velocity = initial_velocity + gravity * elapsed + carried_velocity;
  }

  const Eigen::Matrix3d alignment =
      Eigen::Quaterniond::FromTwoVectors(gravity.normalized(),
                                         Eigen::Vector3d(0.0, 0.0, -1.0))
          .toRotationMatrix();
  const Eigen::Vector3d origin = (*states)[0].pose.translation();
  const Eigen::Vector3d aligned_gravity = alignment * gravity;
  for (State& state : *states) {
    const Eigen::Vector3d position =
        alignment * (state.pose.translation() - origin) + origin;
    const Eigen::Matrix3d rotation =
        alignment * state.pose.rotation().toRotationMatrix();
    state.pose = transform::Rigid3d(position, Eigen::Quaterniond(rotation));
    state.velocity = alignment * state.velocity;
    state.gravity = aligned_gravity;
  }
  return true;
}

}  // namespace lidar
}  // namespace frontend
}  // namespace argus

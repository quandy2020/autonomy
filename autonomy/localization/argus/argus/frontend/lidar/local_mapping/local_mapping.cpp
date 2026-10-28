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

#include "frontend/lidar/local_mapping/local_mapping.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <unordered_map>
#include <utility>

#include "common/math/so3.hpp"
#include "map/voxel/voxel_key_hash.hpp"

namespace argus {
namespace frontend {
namespace lidar {
namespace {

struct BodyCluster {
  Eigen::Matrix3d outer = Eigen::Matrix3d::Zero();
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  int count = 0;
};

struct ImuDelta {
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  double dt = 0.0;
  bool valid = false;
};

map::voxel::VoxelKey WorldKey(const Eigen::Vector3d& point, double voxel_size) {
  map::voxel::VoxelKey key;
  const double coordinates[3] = {point.x(), point.y(), point.z()};
  std::int64_t* slots[3] = {&key.x, &key.y, &key.z};
  for (int axis = 0; axis < 3; ++axis) {
    double loc = coordinates[axis] / voxel_size;
    if (loc < 0.0) {
      loc -= 1.0;
    }
    *slots[axis] = static_cast<std::int64_t>(loc);
  }
  return key;
}

ImuDelta IntegrateImu(const std::vector<sensor::ImuData>& imu,
                      const Eigen::Vector3d& gyro_bias,
                      const Eigen::Vector3d& accel_bias) {
  ImuDelta delta;
  if (imu.size() < 2) {
    return delta;
  }
  for (std::size_t index = 1; index < imu.size(); ++index) {
    const double dt = imu[index].time - imu[index - 1].time;
    if (dt <= 0.0) {
      return ImuDelta{};
    }
    const Eigen::Vector3d gyro =
        0.5 * (imu[index].angular_velocity + imu[index - 1].angular_velocity) -
        gyro_bias;
    const Eigen::Vector3d acceleration =
        0.5 * (imu[index].linear_acceleration +
               imu[index - 1].linear_acceleration) -
        accel_bias;
    const Eigen::Matrix3d rotation = delta.rotation;
    delta.position +=
        delta.velocity * dt + 0.5 * rotation * acceleration * dt * dt;
    delta.velocity += rotation * acceleration * dt;
    delta.rotation = rotation * math::Exp(gyro, dt);
    delta.dt += dt;
  }
  delta.valid = delta.dt > 0.0;
  return delta;
}

}  // namespace

void LocalMapping::Add(const State& state) {
  if (config_.window_size == 0) {
    return;
  }
  window_.push_back(state);
  while (window_.size() > config_.window_size) {
    window_.pop_front();
  }
}

std::vector<backend::RelativePoseFactor> LocalMapping::Push(
    common::KeyframeId id, const State& state,
    const sensor::TimedPointCloud& cloud, const PlaneMap& map,
    const std::vector<sensor::ImuData>& imu_from_previous) {
  std::vector<backend::RelativePoseFactor> factors;
  if (config_.window_size == 0) {
    return factors;
  }
  WindowScan scan;
  scan.id = id;
  scan.state = state;
  scan.cloud = cloud;
  scan.imu_from_previous = imu_from_previous;
  scans_.push_back(std::move(scan));
  if (scans_.size() < config_.window_size) {
    return factors;
  }
  Optimize(map);
  if (scans_.size() <= config_.window_size) {
    return factors;
  }
  const WindowScan dropped = std::move(scans_.front());
  scans_.pop_front();
  const WindowScan& kept = scans_.front();
  if (dropped.id == common::kInvalidKeyframeId ||
      kept.id == common::kInvalidKeyframeId) {
    return factors;
  }
  backend::RelativePoseFactor factor;
  factor.from_id = dropped.id;
  factor.to_id = kept.id;
  factor.relative_pose = dropped.state.pose.inverse() * kept.state.pose;
  factor.scale = 1.0;
  factor.kind = backend::PoseFactorKind::kPose3;
  factors.push_back(factor);
  return factors;
}

void LocalMapping::Optimize(const PlaneMap& map) {
  (void)map;
  const int count = static_cast<int>(scans_.size());
  if (count < 2 || config_.voxel_size <= 0.0) {
    return;
  }
  const int free = count - 1;
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  for (int iteration = 0; iteration < config_.max_iterations; ++iteration) {
    std::unordered_map<map::voxel::VoxelKey, std::vector<BodyCluster>,
                       map::voxel::VoxelKeyHash>
        voxels;
    std::vector<Eigen::Matrix3d> rotation(static_cast<std::size_t>(count));
    std::vector<Eigen::Vector3d> position(static_cast<std::size_t>(count));
    for (int index = 0; index < count; ++index) {
      rotation[static_cast<std::size_t>(index)] =
          scans_[static_cast<std::size_t>(index)]
              .state.pose.rotation()
              .toRotationMatrix();
      position[static_cast<std::size_t>(index)] =
          scans_[static_cast<std::size_t>(index)].state.pose.translation();
      for (const sensor::TimedRangefinderPoint& point :
           scans_[static_cast<std::size_t>(index)].cloud) {
        const Eigen::Vector3d body = point.position.cast<double>();
        const Eigen::Vector3d world =
            rotation[static_cast<std::size_t>(index)] * body +
            position[static_cast<std::size_t>(index)];
        std::vector<BodyCluster>& clusters =
            voxels[WorldKey(world, config_.voxel_size)];
        if (clusters.empty()) {
          clusters.resize(static_cast<std::size_t>(count));
        }
        BodyCluster& cluster = clusters[static_cast<std::size_t>(index)];
        cluster.outer += body * body.transpose();
        cluster.sum += body;
        ++cluster.count;
      }
    }

    Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(6 * free, 6 * free);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(6 * free);
    for (const auto& entry : voxels) {
      Eigen::Matrix3d world_outer = Eigen::Matrix3d::Zero();
      Eigen::Vector3d world_sum = Eigen::Vector3d::Zero();
      int world_count = 0;
      for (int index = 0; index < count; ++index) {
        const BodyCluster& cluster =
            entry.second[static_cast<std::size_t>(index)];
        if (cluster.count == 0) {
          continue;
        }
        const Eigen::Matrix3d& pose_rotation =
            rotation[static_cast<std::size_t>(index)];
        const Eigen::Vector3d& pose_position =
            position[static_cast<std::size_t>(index)];
        const Eigen::Matrix3d cross =
            pose_rotation * cluster.sum * pose_position.transpose();
        world_outer +=
            pose_rotation * cluster.outer * pose_rotation.transpose() + cross +
            cross.transpose() +
            cluster.count * pose_position * pose_position.transpose();
        world_sum +=
            pose_rotation * cluster.sum + cluster.count * pose_position;
        world_count += cluster.count;
      }
      if (world_count <= config_.min_points) {
        continue;
      }
      const double inverse_count = 1.0 / static_cast<double>(world_count);
      const Eigen::Vector3d mean = world_sum * inverse_count;
      const Eigen::Matrix3d covariance =
          world_outer * inverse_count - mean * mean.transpose();
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
      const Eigen::Vector3d eigenvalues = solver.eigenvalues();
      if (eigenvalues[2] <= 1e-12 ||
          std::abs(eigenvalues[0] - eigenvalues[1]) < 1e-8) {
        continue;
      }
      if (eigenvalues[0] >= config_.min_eigenvalue ||
          eigenvalues[0] / eigenvalues[2] >= config_.plane_ratio) {
        continue;
      }
      const Eigen::Vector3d normal = solver.eigenvectors().col(0);
      for (int index = 1; index < count; ++index) {
        const int offset = (index - 1) * 6;
        const Eigen::Matrix3d& pose_rotation =
            rotation[static_cast<std::size_t>(index)];
        for (const sensor::TimedRangefinderPoint& point :
             scans_[static_cast<std::size_t>(index)].cloud) {
          const Eigen::Vector3d body = point.position.cast<double>();
          const Eigen::Vector3d world =
              pose_rotation * body + position[static_cast<std::size_t>(index)];
          if (!(WorldKey(world, config_.voxel_size) == entry.first)) {
            continue;
          }
          const double residual = normal.dot(world - mean);
          Eigen::Matrix<double, 6, 1> jacobian;
          jacobian.head<3>() =
              math::SkewSymmetric(body) * pose_rotation.transpose() * normal;
          jacobian.tail<3>() = normal;
          hessian.block<6, 6>(offset, offset) +=
              jacobian * jacobian.transpose();
          gradient.segment<6>(offset) += jacobian * residual;
        }
      }
    }

    for (int index = 1; index < count; ++index) {
      const ImuDelta delta = IntegrateImu(
          scans_[static_cast<std::size_t>(index)].imu_from_previous,
          scans_[static_cast<std::size_t>(index - 1)].state.gyro_bias,
          scans_[static_cast<std::size_t>(index - 1)].state.accel_bias);
      if (!delta.valid) {
        continue;
      }
      const State& first = scans_[static_cast<std::size_t>(index - 1)].state;
      const State& second = scans_[static_cast<std::size_t>(index)].state;
      const Eigen::Matrix3d first_rotation =
          rotation[static_cast<std::size_t>(index - 1)];
      const Eigen::Vector3d displacement =
          position[static_cast<std::size_t>(index)] -
          position[static_cast<std::size_t>(index - 1)] -
          first.velocity * delta.dt - 0.5 * delta.dt * delta.dt * first.gravity;
      const Eigen::Vector3d translation_residual =
          first_rotation.transpose() * displacement - delta.position;
      const Eigen::Matrix3d rotation_error =
          delta.rotation.transpose() * first_rotation.transpose() *
          rotation[static_cast<std::size_t>(index)];
      const Eigen::Vector3d rotation_residual = math::Log(rotation_error);
      Eigen::Matrix<double, 6, 6> imu_hessian =
          Eigen::Matrix<double, 6, 6>::Zero();
      Eigen::Matrix<double, 6, 1> imu_gradient =
          Eigen::Matrix<double, 6, 1>::Zero();
      Eigen::Matrix<double, 3, 6> translation_jacobian =
          Eigen::Matrix<double, 3, 6>::Zero();
      translation_jacobian.rightCols<3>() = first_rotation.transpose();
      Eigen::Matrix<double, 3, 6> rotation_jacobian =
          Eigen::Matrix<double, 3, 6>::Zero();
      rotation_jacobian.leftCols<3>() = identity;
      imu_hessian += translation_jacobian.transpose() * translation_jacobian +
                     rotation_jacobian.transpose() * rotation_jacobian;
      imu_gradient += translation_jacobian.transpose() * translation_residual +
                      rotation_jacobian.transpose() * rotation_residual;
      if (index > 1) {
        Eigen::Matrix<double, 3, 6> previous_translation =
            Eigen::Matrix<double, 3, 6>::Zero();
        previous_translation.leftCols<3>() =
            math::SkewSymmetric(first_rotation.transpose() * displacement);
        previous_translation.rightCols<3>() = -first_rotation.transpose();
        Eigen::Matrix<double, 3, 6> previous_rotation =
            Eigen::Matrix<double, 3, 6>::Zero();
        previous_rotation.leftCols<3>() = -identity;
        const int previous = (index - 2) * 6;
        const int current = (index - 1) * 6;
        hessian.block<6, 6>(previous, previous) +=
            config_.imu_weight *
            (previous_translation.transpose() * previous_translation +
             previous_rotation.transpose() * previous_rotation);
        hessian.block<6, 6>(current, previous) +=
            config_.imu_weight *
            (translation_jacobian.transpose() * previous_translation +
             rotation_jacobian.transpose() * previous_rotation);
        hessian.block<6, 6>(previous, current) +=
            hessian.block<6, 6>(current, previous).transpose();
        gradient.segment<6>(previous) +=
            config_.imu_weight *
            (previous_translation.transpose() * translation_residual +
             previous_rotation.transpose() * rotation_residual);
      }
      const int offset = (index - 1) * 6;
      hessian.block<6, 6>(offset, offset) += config_.imu_weight * imu_hessian;
      gradient.segment<6>(offset) += config_.imu_weight * imu_gradient;
    }

    hessian.diagonal().array() += 1e-8;
    const Eigen::VectorXd step = hessian.ldlt().solve(-gradient);
    if (!step.allFinite()) {
      break;
    }
    double largest_rotation = 0.0;
    double largest_translation = 0.0;
    for (int index = 1; index < count; ++index) {
      const Eigen::Matrix<double, 6, 1> delta =
          step.segment<6>((index - 1) * 6);
      State& state = scans_[static_cast<std::size_t>(index)].state;
      const Eigen::Matrix3d updated =
          rotation[static_cast<std::size_t>(index)] *
          math::Exp(delta.head<3>());
      state.pose = transform::Rigid3d(
          position[static_cast<std::size_t>(index)] + delta.tail<3>(),
          Eigen::Quaterniond(updated));
      largest_rotation = std::max(largest_rotation, delta.head<3>().norm());
      largest_translation =
          std::max(largest_translation, delta.tail<3>().norm());
    }
    if (largest_rotation < 1e-8 && largest_translation < 1e-8) {
      break;
    }
  }
}

}  // namespace lidar
}  // namespace frontend
}  // namespace argus

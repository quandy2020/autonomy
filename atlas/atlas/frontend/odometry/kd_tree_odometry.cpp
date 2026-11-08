/*
 * Copyright 2026 The Openbot Authors
 */
#include "frontend/odometry/kd_tree_odometry.hpp"

#include "common/math/so3.hpp"
#include "frontend/odometry/downsample.hpp"

namespace atlas {
namespace frontend {
namespace odometry {

using ::atlas::frontend::inertial::InertialState;
using ::atlas::frontend::inertial::kInertialStateDimension;
using ::atlas::frontend::inertial::VectorList;

void KdTreeOdometry::Clear() {
  map_points_.clear();
  map_point_cloud_.clear();
}

bool KdTreeOdometry::AddPointsOrCheckReadiness(
    const PointObservationList& point_observations,
    const InertialState& inertial_state) {
  if (map_points_.size() < 100) {
    for (const PointObservation& point_observation : point_observations) {
      const Eigen::Vector3d world_point =
          inertial_state.R * point_observation.position + inertial_state.p;
      map_points_.push_back(world_point);
      sensor::PointType neighbor_point;
      neighbor_point.x = static_cast<float>(world_point[0]);
      neighbor_point.y = static_cast<float>(world_point[1]);
      neighbor_point.z = static_cast<float>(world_point[2]);
      map_point_cloud_.push_back(neighbor_point);
    }
    nearest_neighbor_index_.SetInput(map_points_);
    return false;
  }
  return true;
}

bool KdTreeOdometry::EstimatePose(
    const SharedPointObservationList& point_observations,
    InertialState* inertial_state) {
  if (!point_observations || inertial_state == nullptr ||
      map_points_.size() < 100) {
    return false;
  }
  const InertialState predicted_state = *inertial_state;
  constexpr int kMaximumIterationCount = 4;
  Eigen::Matrix<double, kInertialStateDimension, kInertialStateDimension>
      kalman_gain = Eigen::Matrix<double, kInertialStateDimension,
                                  kInertialStateDimension>::Zero();
  Eigen::Matrix<double, kInertialStateDimension, kInertialStateDimension>
      state_information_matrix = Eigen::Matrix<double, kInertialStateDimension,
                                               kInertialStateDimension>::Zero();
  const Eigen::Matrix<double, kInertialStateDimension, kInertialStateDimension>
      identity_matrix = Eigen::Matrix<double, kInertialStateDimension,
                                      kInertialStateDimension>::Identity();
  const Eigen::Matrix<double, kInertialStateDimension, kInertialStateDimension>
      inverse_state_covariance = inertial_state->cov.inverse();

  Eigen::Matrix<double, kNearestNeighborCount, 1> plane_constraint_targets =
      Eigen::Matrix<double, kNearestNeighborCount, 1>::Ones() * -1.0;
  const int point_count = static_cast<int>(point_observations->size());
  std::vector<double> plane_offsets(static_cast<size_t>(point_count), -1.0);
  VectorList<3> plane_normals(static_cast<size_t>(point_count));
  bool should_recompute_associations = true;
  int association_refresh_count = 0;
  bool has_converged = false;

  for (int iteration_index = 0; iteration_index < kMaximumIterationCount;
       ++iteration_index) {
    Eigen::Matrix<double, 6, 6> measurement_information_matrix =
        Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> measurement_gradient =
        Eigen::Matrix<double, 6, 1>::Zero();
    int valid_association_count = 0;
    for (int i = 0; i < point_count; ++i) {
      PointObservation& point_observation =
          (*point_observations)[static_cast<size_t>(i)];
      const Eigen::Matrix3d point_skew_symmetric =
          math::SkewSymmetric(point_observation.position);
      const Eigen::Vector3d world_point =
          inertial_state->R * point_observation.position + inertial_state->p;
      if (should_recompute_associations) {
        std::vector<int> neighbor_indices;
        std::vector<float> squared_distances;
        if (nearest_neighbor_index_.NearestKSearch(
                world_point, kNearestNeighborCount, &neighbor_indices,
                &squared_distances) < kNearestNeighborCount) {
          plane_offsets[static_cast<size_t>(i)] = -1;
          continue;
        }
        Eigen::Matrix<double, kNearestNeighborCount, 3> neighbor_points;
        for (int k = 0; k < kNearestNeighborCount; ++k) {
          const Eigen::Vector3d& neighbor_point =
              map_points_[static_cast<size_t>(
                  neighbor_indices[static_cast<size_t>(k)])];
          neighbor_points.row(k) << neighbor_point.x(), neighbor_point.y(),
              neighbor_point.z();
        }
        Eigen::Vector3d plane_coefficients =
            neighbor_points.colPivHouseholderQr().solve(
                plane_constraint_targets);
        bool invalid_plane_fit = false;
        for (int k = 0; k < kNearestNeighborCount; ++k) {
          if (std::fabs(plane_coefficients.dot(neighbor_points.row(k)) + 1.0) >
              0.1) {
            invalid_plane_fit = true;
          }
        }
        if (invalid_plane_fit) {
          plane_offsets[static_cast<size_t>(i)] = -1;
          continue;
        }
        const double plane_offset = 1.0 / plane_coefficients.norm();
        plane_offsets[static_cast<size_t>(i)] = plane_offset;
        plane_normals[static_cast<size_t>(i)] =
            plane_coefficients * plane_offset;
      }
      if (plane_offsets[static_cast<size_t>(i)] >= 0) {
        const double signed_plane_distance =
            plane_normals[static_cast<size_t>(i)].dot(world_point) +
            plane_offsets[static_cast<size_t>(i)];
        Eigen::Matrix<double, 6, 1> state_jacobian;
        state_jacobian.head(3) = point_skew_symmetric *
                                 inertial_state->R.transpose() *
                                 plane_normals[static_cast<size_t>(i)];
        state_jacobian.tail(3) = plane_normals[static_cast<size_t>(i)];
        measurement_information_matrix +=
            state_jacobian * state_jacobian.transpose();
        measurement_gradient += state_jacobian * (-signed_plane_distance);
        ++valid_association_count;
      }
    }
    if (valid_association_count < 20) {
      return false;
    }
    state_information_matrix.block<6, 6>(0, 0) = measurement_information_matrix;
    const Eigen::Matrix<double, kInertialStateDimension,
                        kInertialStateDimension>
        posterior_covariance =
            (state_information_matrix + inverse_state_covariance / 1000.0)
                .inverse();
    kalman_gain.block<kInertialStateDimension, 6>(0, 0) =
        posterior_covariance.block<kInertialStateDimension, 6>(0, 0) *
        measurement_information_matrix;
    const Eigen::Matrix<double, kInertialStateDimension, 1> state_difference =
        predicted_state - *inertial_state;
    Eigen::Matrix<double, kInertialStateDimension, 1> solution =
        posterior_covariance.block<kInertialStateDimension, 6>(0, 0) *
            measurement_gradient +
        state_difference -
        kalman_gain.block<kInertialStateDimension, 6>(0, 0) *
            state_difference.block<6, 1>(0, 0);
    *inertial_state += solution;
    const Eigen::Vector3d rotation_update = solution.block<3, 1>(0, 0);
    const Eigen::Vector3d translation_update = solution.block<3, 1>(3, 0);
    should_recompute_associations = false;
    if ((rotation_update.norm() * 57.3 < 0.01) &&
        (translation_update.norm() * 100 < 0.015)) {
      should_recompute_associations = true;
      has_converged = true;
      ++association_refresh_count;
    }
    if (iteration_index == kMaximumIterationCount - 2 && !has_converged) {
      should_recompute_associations = true;
    }
    if (association_refresh_count >= 2 ||
        iteration_index == kMaximumIterationCount - 1) {
      inertial_state->cov =
          (identity_matrix - kalman_gain) * inertial_state->cov;
      break;
    }
  }

  for (const PointObservation& point_observation : *point_observations) {
    const Eigen::Vector3d world_point =
        inertial_state->R * point_observation.position + inertial_state->p;
    map_points_.push_back(world_point);
    sensor::PointType map_point;
    map_point.x = static_cast<float>(world_point[0]);
    map_point.y = static_cast<float>(world_point[1]);
    map_point.z = static_cast<float>(world_point[2]);
    map_point_cloud_.push_back(map_point);
  }
  DownsampleVoxel(&map_point_cloud_, 0.5);
  map_points_.clear();
  map_points_.reserve(map_point_cloud_.size());
  for (const auto& p : map_point_cloud_.points) {
    map_points_.emplace_back(p.x, p.y, p.z);
  }
  nearest_neighbor_index_.SetInput(map_points_);
  return true;
}

}  // namespace odometry
}  // namespace frontend
}  // namespace atlas

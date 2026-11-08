/*
 * Copyright 2026 The Openbot Authors
 */
#include "frontend/initialization/initialization.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>

#include "common/math/so3.hpp"
#include "frontend/local_mapping/bundle_adjustment_support.hpp"
#include "frontend/local_mapping/gravity_constrained_bundle_adjuster.hpp"
#include "frontend/odometry/point_covariance_estimator.hpp"
#include "glog/logging.h"
#include "map/sliding_voxel_map_operations.hpp"

namespace atlas {
namespace frontend {
namespace initialization {

using ::atlas::map::PointObservation;
using ::atlas::map::PointObservationList;
using ::atlas::map::SharedPointObservationList;

using ::atlas::frontend::inertial::InertialPreintegrationOptions;
using ::atlas::frontend::inertial::InertialPreintegrator;
using ::atlas::frontend::inertial::InertialState;
using ::atlas::frontend::local_mapping::GravityConstrainedBundleAdjuster;
using ::atlas::frontend::local_mapping::MakePoseWindow;
using ::atlas::frontend::odometry::PointCovarianceEstimator;

namespace {

bool ValidateInitializationInput(const MotionInitializationInput& input,
                                 const MotionInitializationWorkspace& workspace,
                                 MotionInitializationResult* result) {
  if (workspace.frame_window_partitions.empty() ||
      workspace.map_options == nullptr || workspace.local_window_size <= 0) {
    return false;
  }
  const std::size_t window_size =
      static_cast<std::size_t>(workspace.local_window_size);
  if (input.lidar_scans.size() < window_size ||
      workspace.state_window.size() < window_size ||
      workspace.point_observation_window.size() < window_size ||
      input.imu_samples_by_scan.size() < window_size ||
      input.scan_start_times.size() < window_size) {
    result->status = MotionInitializationStatus::kInsufficientScans;
    return false;
  }
  for (std::size_t index = 0; index < window_size; ++index) {
    if (input.lidar_scans[index] == nullptr) {
      return false;
    }
  }
  return true;
}

std::shared_ptr<SlidingVoxelMapOptions> MakeInitializationMapOptions(
    const MotionInitializationWorkspace& workspace) {
  auto options =
      std::make_shared<SlidingVoxelMapOptions>(*workspace.map_options);
  options->minimum_plane_eigenvalue = 0.02;
  for (double& threshold : options->plane_eigenvalue_thresholds) {
    threshold = 1.0 / 4.0;
  }
  return options;
}

void PrepareInitializationObservations(
    const MotionInitializationInput& input,
    MotionInitializationWorkspace& workspace,
    const std::shared_ptr<SlidingVoxelMapOptions>& map_options,
    int convergence_flag) {
  for (int window_index = 0; window_index < workspace.local_window_size;
       ++window_index) {
    const std::size_t index = static_cast<std::size_t>(window_index);
    if (!workspace.point_observation_window[index]) {
      workspace.point_observation_window[index] =
          std::make_shared<PointObservationList>();
    }
    PointObservationList& observations =
        *workspace.point_observation_window[index];
    observations.clear();
    const int previous_index = window_index == 0 ? 0 : window_index - 1;
    UndistortInitializationScan(
        *input.lidar_scans[index], &observations, workspace.state_window[index],
        workspace.state_window[static_cast<std::size_t>(previous_index)],
        input.imu_samples_by_scan[index], input.scan_start_times[index],
        input.lidar_to_imu_extrinsic, input.point_time_unavailable,
        workspace.gravity_scale);
    if (convergence_flag == 0) {
      continue;
    }
    for (PointObservation& observation : observations) {
      PointCovarianceEstimator::CalculateBodyPointCovariance(
          observation.position, static_cast<float>(input.range_noise_meters),
          static_cast<float>(input.beam_angle_noise_degrees),
          &observation.covariance);
    }
  }
  if (convergence_flag == 1) {
    map_options->minimum_plane_eigenvalue =
        workspace.map_options->minimum_plane_eigenvalue;
    map_options->plane_eigenvalue_thresholds =
        workspace.map_options->plane_eigenvalue_thresholds;
  }
}

std::vector<double> OptimizeInitializationWindow(
    const MotionInitializationInput& input,
    MotionInitializationWorkspace& workspace,
    const InertialPreintegrationOptions& preintegration_options) {
  GravityConstrainedBundleAdjuster optimizer(
      workspace.inertial_factor_coefficient, workspace.task_executor,
      workspace.worker_thread_count);
  std::deque<InertialPreintegrator*> preintegrator_view;
  for (const auto& preintegrator : workspace.inertial_preintegrators) {
    preintegrator_view.push_back(preintegrator.get());
  }
  std::vector<double> residual_history;
  optimizer.Optimize(workspace.state_window, workspace.lidar_factor,
                     preintegrator_view, residual_history, &workspace.hessian,
                     3);
  workspace.inertial_preintegrators.clear();
  for (int index = 1; index < workspace.local_window_size; ++index) {
    auto preintegrator = std::make_unique<InertialPreintegrator>(
        workspace.state_window[static_cast<std::size_t>(index - 1)].bg,
        workspace.state_window[static_cast<std::size_t>(index - 1)].ba,
        preintegration_options);
    preintegrator->PushImu(
        input.imu_samples_by_scan[static_cast<std::size_t>(index)]);
    workspace.inertial_preintegrators.push_back(std::move(preintegrator));
  }
  return residual_history;
}

Eigen::Vector3d CalculatePlaneObservability(
    const PointCloudFactor& lidar_factor) {
  Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Zero();
  for (const Eigen::Matrix3d& eigenvectors : lidar_factor.eigenvectors) {
    const Eigen::Vector3d plane_normal = eigenvectors.col(0);
    information_matrix += plane_normal * plane_normal.transpose();
  }
  return Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(information_matrix)
      .eigenvalues();
}

bool HasInitializationConverged(const std::vector<double>& residual_history,
                                int iteration_index,
                                double convergence_threshold) {
  return residual_history.size() >= 2 && iteration_index >= 2 &&
         std::fabs(residual_history[0] - residual_history[1]) /
                 std::max(residual_history[0], 1e-9) <
             convergence_threshold;
}

void SetInitializationStatus(MotionInitializationResult* result,
                             int convergence_flag, bool has_constraint_factors,
                             bool has_observability_estimate,
                             bool is_degraded) {
  if (convergence_flag == 1) {
    result->status = MotionInitializationStatus::kInitialized;
  } else if (!has_constraint_factors) {
    result->status = MotionInitializationStatus::kInsufficientConstraints;
  } else if (!has_observability_estimate) {
    result->status = MotionInitializationStatus::kNotConverged;
  } else if (is_degraded || result->gravity_magnitude < 9.6 ||
             result->gravity_magnitude > 10.0) {
    result->status = MotionInitializationStatus::kDegraded;
  } else {
    result->status = MotionInitializationStatus::kNotConverged;
  }
}

}  // namespace

bool AlignGravity(const std::deque<sensor::ImuData>& imu_samples,
                  InertialState* x, double scale_gravity) {
  if (x == nullptr || imu_samples.size() < 10) {
    return false;
  }
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& imu : imu_samples) {
    mean += imu.linear_acceleration;
  }
  mean /= static_cast<double>(imu_samples.size());
  x->SetZero();
  x->g = -mean * scale_gravity;
  return true;
}

bool HasEnoughMotionInitializationScans(int buffered_scan_count,
                                        int minimum_scan_count) {
  return buffered_scan_count >= minimum_scan_count;
}

bool DiscardOldestMotionInitializationScan(MotionInitializationInput* input) {
  if (input == nullptr || input->lidar_scans.empty() ||
      input->lidar_scans.size() != input->imu_samples_by_scan.size() ||
      input->lidar_scans.size() != input->scan_start_times.size()) {
    return false;
  }
  input->lidar_scans.erase(input->lidar_scans.begin());
  input->imu_samples_by_scan.erase(input->imu_samples_by_scan.begin());
  input->scan_start_times.erase(input->scan_start_times.begin());
  return true;
}

bool RebaseMotionInitializationWindow(
    std::vector<InertialState>* state_window,
    const InertialState& pose_anchor) {
  if (state_window == nullptr || state_window->empty()) {
    return false;
  }
  const InertialState& latest_state = state_window->back();
  const Eigen::Matrix3d rotation_delta =
      pose_anchor.R * latest_state.R.transpose();
  const Eigen::Vector3d position_delta =
      pose_anchor.p - rotation_delta * latest_state.p;
  for (InertialState& state : *state_window) {
    state.R = rotation_delta * state.R;
    state.p = rotation_delta * state.p + position_delta;
    state.v = rotation_delta * state.v;
    state.g = rotation_delta * state.g;
  }
  return true;
}

bool AlignStateWindowWithGravity(std::vector<InertialState>* state_window) {
  if (state_window == nullptr || state_window->empty()) {
    return false;
  }
  Eigen::Vector3d g0 = (*state_window)[0].g;
  const double gravity_magnitude = g0.norm();
  if (!g0.allFinite() || !std::isfinite(gravity_magnitude) ||
      gravity_magnitude <= 1e-6) {
    return false;
  }
  Eigen::Vector3d n0 = g0 / gravity_magnitude;
  Eigen::Vector3d n1(0, 0, 1);
  if (n0[2] < 0) {
    n1[2] = -1;
  }
  Eigen::Vector3d rotvec = n0.cross(n1);
  const double rnorm = rotvec.norm();
  if (rnorm < 1e-12) {
    return true;
  }
  rotvec /= rnorm;
  const Eigen::AngleAxisd angaxis(std::asin(std::min(1.0, rnorm)), rotvec);
  const Eigen::Matrix3d rot = angaxis.matrix();
  g0 = rot * g0;
  const Eigen::Vector3d p0 = (*state_window)[0].p;
  for (InertialState& x : *state_window) {
    x.p = rot * (x.p - p0) + p0;
    x.R = rot * x.R;
    x.v = rot * x.v;
    x.g = g0;
  }
  return true;
}

void UndistortInitializationScan(const sensor::PointCloudXYZI& point_cloud,
                                 PointObservationList* point_observations,
                                 InertialState current_state,
                                 const InertialState& previous_state,
                                 const std::deque<sensor::ImuData>& imu_samples,
                                 double scan_start_time,
                                 const InertialState& lidar_to_imu_extrinsic,
                                 bool point_time_unavailable,
                                 double gravity_scale) {
  if (point_observations == nullptr) {
    return;
  }
  point_observations->clear();
  current_state.bg = previous_state.bg;
  current_state.ba = previous_state.ba;
  Eigen::Vector3d imu_acceleration;
  Eigen::Vector3d average_angular_velocity;
  Eigen::Vector3d average_acceleration;
  Eigen::Vector3d imu_velocity(current_state.v);
  Eigen::Vector3d imu_position(current_state.p);
  Eigen::Matrix3d imu_rotation(current_state.R);
  std::vector<InertialState> inertial_poses;

  if (imu_samples.size() < 2) {
    PointObservation point_observation;
    point_observation.covariance.setIdentity();
    for (const sensor::PointType& point : point_cloud.points) {
      point_observation.position << point.x, point.y, point.z;
      point_observation.position =
          lidar_to_imu_extrinsic.R * point_observation.position +
          lidar_to_imu_extrinsic.p;
      point_observations->push_back(point_observation);
    }
    return;
  }

  for (auto it = imu_samples.end() - 1; it != imu_samples.begin(); --it) {
    const sensor::ImuData& head = *(it - 1);
    const sensor::ImuData& tail = *it;
    average_angular_velocity =
        0.5 * (head.angular_velocity + tail.angular_velocity);
    average_acceleration =
        0.5 * (head.linear_acceleration + tail.linear_acceleration);
    average_angular_velocity -= current_state.bg;
    average_acceleration =
        average_acceleration * gravity_scale - current_state.ba;
    const double dt = head.time - tail.time;
    const Eigen::Matrix3d rotation_increment =
        math::Exp(average_angular_velocity, dt);
    imu_acceleration = imu_rotation * average_acceleration + current_state.g;
    imu_position =
        imu_position + imu_velocity * dt + 0.5 * imu_acceleration * dt * dt;
    imu_velocity = imu_velocity + imu_acceleration * dt;
    imu_rotation = imu_rotation * rotation_increment;
    const double relative_scan_time = head.time - scan_start_time;
    inertial_poses.emplace_back(relative_scan_time, imu_rotation, imu_position,
                                imu_velocity, average_angular_velocity,
                                imu_acceleration);
  }

  PointObservation point_observation;
  point_observation.covariance.setIdentity();
  if (point_time_unavailable) {
    for (const sensor::PointType& point : point_cloud.points) {
      point_observation.position << point.x, point.y, point.z;
      point_observation.position =
          lidar_to_imu_extrinsic.R * point_observation.position +
          lidar_to_imu_extrinsic.p;
      point_observations->push_back(point_observation);
    }
    return;
  }
  if (point_cloud.empty() || inertial_poses.empty()) {
    return;
  }
  auto point_iterator = point_cloud.points.end() - 1;
  for (InertialState& head : inertial_poses) {
    imu_rotation = head.R;
    imu_acceleration = head.ba;
    imu_velocity = head.v;
    imu_position = head.p;
    average_angular_velocity = head.bg;
    for (; point_iterator->curvature > head.t; --point_iterator) {
      const double dt = point_iterator->curvature - head.t;
      const Eigen::Matrix3d point_rotation =
          imu_rotation * math::Exp(average_angular_velocity, dt);
      const Eigen::Vector3d imu_relative_translation =
          imu_position + imu_velocity * dt + 0.5 * imu_acceleration * dt * dt -
          current_state.p;
      const Eigen::Vector3d lidar_point(point_iterator->x, point_iterator->y,
                                        point_iterator->z);
      point_observation.position =
          current_state.R.transpose() *
          (point_rotation * (lidar_to_imu_extrinsic.R * lidar_point +
                             lidar_to_imu_extrinsic.p) +
           imu_relative_translation);
      point_observations->push_back(point_observation);
      if (point_iterator == point_cloud.points.begin()) {
        break;
      }
    }
  }
}

MotionInitializationResult MotionInitializer::Run(
    const MotionInitializationInput& input,
    MotionInitializationWorkspace& workspace) const {
  MotionInitializationResult result;
  if (!ValidateInitializationInput(input, workspace, &result)) {
    return result;
  }
  int convergence_flag = 0;
  auto initialization_map_options = MakeInitializationMapOptions(workspace);
  InertialPreintegrationOptions preintegration_options =
      workspace.preintegration_options;
  preintegration_options.gravity_scale = workspace.gravity_scale;
  double convergence_threshold = 0.05;
  bool is_degraded = true;
  bool has_constraint_factors = false;
  bool has_plane_observability_estimate = false;
  Eigen::Vector3d observability_eigenvalues = Eigen::Vector3d::Zero();

  for (int iteration_index = 0; iteration_index < 10; ++iteration_index) {
    result.iteration_count = iteration_index + 1;
    PrepareInitializationObservations(
        input, workspace, initialization_map_options, convergence_flag);
    workspace.map_operations.BuildInitializationMap(
        &workspace.surface_map, &workspace.sliding_surface_map,
        MakePoseWindow(workspace.state_window),
        workspace.point_observation_window, workspace.local_window_size,
        workspace.local_window_size, &workspace.lidar_factor,
        &workspace.frame_window_partitions, initialization_map_options);
    if (workspace.lidar_factor.point_clusters_by_window.size() < 10) {
      break;
    }
    has_constraint_factors = true;
    const std::vector<double> residual_history =
        OptimizeInitializationWindow(input, workspace, preintegration_options);
    if (HasInitializationConverged(residual_history, iteration_index,
                                   convergence_threshold)) {
      observability_eigenvalues =
          CalculatePlaneObservability(workspace.lidar_factor);
      has_plane_observability_estimate = true;
      is_degraded = observability_eigenvalues[0] <
                    workspace.minimum_plane_observability_eigenvalue;
      convergence_threshold = 0.01;
      if (convergence_flag == 0) {
        if (!AlignStateWindowWithGravity(&workspace.state_window)) {
          is_degraded = true;
          break;
        }
        convergence_flag = 1;
        continue;
      }
      break;
    }
  }

  result.estimated_state = workspace.state_window[static_cast<std::size_t>(
      workspace.local_window_size - 1)];
  result.plane_observability_eigenvalues = observability_eigenvalues;
  result.gravity_magnitude = result.estimated_state.g.norm();
  if (is_degraded || result.gravity_magnitude < 9.6 ||
      result.gravity_magnitude > 10.0) {
    convergence_flag = 0;
  }
  if (convergence_flag == 0) {
    workspace.map_operations.ClearMap(&workspace.surface_map,
                                      &workspace.sliding_surface_map,
                                      &workspace.frame_window_partitions[0]);
  }

  VLOG(1) << "motion_init eig=" << observability_eigenvalues.transpose()
          << " gravity=" << result.gravity_magnitude
          << " degraded=" << is_degraded << " flag=" << convergence_flag;
  SetInitializationStatus(&result, convergence_flag, has_constraint_factors,
                          has_plane_observability_estimate, is_degraded);
  return result;
}

}  // namespace initialization
}  // namespace frontend
}  // namespace atlas

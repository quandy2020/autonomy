/*
 * Copyright 2026 The Openbot Authors
 */
#include "frontend/trajectory_builder.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>

#include "backend/global_mapping/global_mapping_optimizer.hpp"
#include "backend/global_mapping/global_mapping_request_builder.hpp"
#include "backend/global_mapping/pose_graph_committer.hpp"
#include "backend/global_mapping/pose_graph_optimization.hpp"
#include "common/math/so3.hpp"
#include "frontend/local_mapping/bundle_adjustment_support.hpp"
#include "frontend/odometry/downsample.hpp"
#include "frontend/odometry/normal_icp.hpp"
#include "frontend/odometry/odometry.hpp"
#include "frontend/odometry/point_covariance_estimator.hpp"
#include "frontend/pose_graph_snapshot.hpp"
#include "glog/logging.h"
#include "map/hierarchical_bundle_adjustment.hpp"
#include "map/keyframe_load.hpp"
#include "map/point_observation_transform.hpp"
#include "map/serialization/voxel_map_serializer.hpp"
#include "sensor/point_cloud_transform.hpp"

namespace atlas {
namespace frontend {

namespace loop_closure = ::atlas::backend::loop_closure;

using ::atlas::map::BundleAdjustmentKeyframe;
using ::atlas::map::FrameWindow;
using ::atlas::map::GlobalBundleAdjustmentConfiguration;
using ::atlas::map::Keyframe;
using ::atlas::map::SlidingVoxelMap;
using ::atlas::map::SlidingVoxelMapOperations;
using ::atlas::map::SlidingVoxelMapOptions;
using ::atlas::map::SlidingVoxelMapView;
using ::atlas::map::SlidingVoxelTree;
using ::atlas::sensor::PointType;

using ::atlas::backend::global_mapping::GlobalMappingOptimizer;
using ::atlas::backend::global_mapping::GlobalMappingRequest;
using ::atlas::backend::global_mapping::GlobalMappingRequestBuilder;
using ::atlas::backend::global_mapping::GlobalMappingResult;
using ::atlas::backend::global_mapping::GlobalMappingStage;
using ::atlas::backend::global_mapping::GlobalMappingStatus;
using ::atlas::backend::global_mapping::OptimizePoseGraph;
using ::atlas::backend::global_mapping::OptimizeTopDownPoseGraph;
using ::atlas::backend::global_mapping::PgoEdges;
using ::atlas::backend::global_mapping::PoseGraphCommitter;
using ::atlas::backend::global_mapping::PreviousSession;
using ::atlas::backend::global_mapping::PreviousSessionLoader;
using ::atlas::backend::global_mapping::PreviousSessionOptions;

using ::atlas::map::FittedPlane;

using ::atlas::frontend::odometry::DownsampleVoxel;
using ::atlas::frontend::odometry::NormalIcp;
using ::atlas::frontend::odometry::PointCovarianceEstimator;
using ::atlas::frontend::odometry::UpdateDegradationCount;
using ::atlas::frontend::odometry::ProcessLidarVendor;
using ::atlas::frontend::odometry::ScanPose;
using ::atlas::frontend::odometry::ScanPosePtr;
using ::atlas::map::PointObservation;
using ::atlas::map::PointObservationList;
using ::atlas::map::SharedPointObservationList;
using ::atlas::map::TransformPointObservationsToWorld;

using ::atlas::frontend::inertial::InertialPreintegrationOptions;
using ::atlas::frontend::inertial::InertialPreintegrator;
using ::atlas::frontend::inertial::InertialState;
using ::atlas::frontend::inertial::kInertialStateDimension;
using ::atlas::frontend::inertial::VectorList;
using ::atlas::frontend::local_mapping::MakePoseWindow;

namespace {

std::vector<Keyframe*> MakeKeyframeView(
    const std::vector<std::unique_ptr<Keyframe>>& keyframe_owners) {
  std::vector<Keyframe*> keyframe_view;
  keyframe_view.reserve(keyframe_owners.size());
  for (const auto& keyframe_owner : keyframe_owners) {
    keyframe_view.push_back(keyframe_owner.get());
  }
  return keyframe_view;
}

sensor::PointCloudXYZI CreateSubmapPointCloud(
    const std::vector<BundleAdjustmentKeyframe>& keyframes,
    const std::vector<InertialState>& optimized_poses, double voxel_size) {
  sensor::PointCloudXYZI submap_cloud;
  if (keyframes.empty() || keyframes.size() != optimized_poses.size()) {
    return submap_cloud;
  }

  const InertialState& reference_pose = optimized_poses.front();
  for (std::size_t keyframe_index = 0; keyframe_index < keyframes.size();
       ++keyframe_index) {
    const auto& keyframe_cloud = keyframes[keyframe_index].cloud;
    if (!keyframe_cloud) {
      continue;
    }
    const InertialState& keyframe_pose = optimized_poses[keyframe_index];
    const Eigen::Matrix3d rotation =
        reference_pose.R.transpose() * keyframe_pose.R;
    const Eigen::Vector3d translation =
        reference_pose.R.transpose() *
        (keyframe_pose.p - reference_pose.p);
    for (const PointType& source_point : keyframe_cloud->points) {
      PointType submap_point = source_point;
      const Eigen::Vector3d point_in_reference =
          rotation * Eigen::Vector3d(source_point.x, source_point.y,
                                     source_point.z) +
          translation;
      submap_point.x = static_cast<float>(point_in_reference.x());
      submap_point.y = static_cast<float>(point_in_reference.y());
      submap_point.z = static_cast<float>(point_in_reference.z());
      submap_point.intensity =
          static_cast<float>(keyframes[keyframe_index].session_id);
      submap_cloud.push_back(submap_point);
    }
  }
  DownsampleVoxel(&submap_cloud, voxel_size / 8.0);
  return submap_cloud;
}

}  // namespace

TrajectoryBuilder::TrajectoryBuilder()
    : TrajectoryBuilder(TrajectoryBuilderOptions{}) {}

TrajectoryBuilder::TrajectoryBuilder(const TrajectoryBuilderOptions& config)
    : config_(config) {}

TrajectoryBuilder::~TrajectoryBuilder() {
  ShutdownGlobalMapping();
  trajectory_lifecycle_.Stop();
  ClearInertialPreintegrators();
  sliding_voxel_map_.clear();
  keyframes_.clear();
  for (PreviousSession& session : previous_sessions_) {
    session.keyframes.clear();
  }
  previous_sessions_.clear();
}

void TrajectoryBuilder::ShutdownGlobalMapping() { global_mapping_.Shutdown(); }

void TrajectoryBuilder::ClearInertialPreintegrators() {
  imu_preintegrators_.clear();
}

bool TrajectoryBuilder::Finalize(std::string* error) {
  {
    std::lock_guard<std::mutex> frontend_lock(frontend_state_mutex_);
    finalizing_ = true;
  }
  trajectory_lifecycle_.BeginFinalization();
  const auto global_mapping_start = std::chrono::steady_clock::now();
  ShutdownGlobalMapping();
  LOG(INFO) << "Global mapping drain completed in "
            << std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                             global_mapping_start)
                   .count()
            << " seconds";
  std::string save_error;
  const bool saved = SaveSession(&save_error);
  const std::string global_mapping_error = global_mapping_.worker_error();
  trajectory_lifecycle_.Stop();
  if (!global_mapping_error.empty()) {
    if (error != nullptr) {
      *error = "global mapping failed: " + global_mapping_error;
      if (!saved) {
        *error += "; session save failed: " + save_error;
      }
    }
    return false;
  }
  if (!saved && error != nullptr) {
    *error = std::move(save_error);
  }
  return saved;
}

sensor::PointCloudXYZI TrajectoryBuilder::GlobalMapSnapshot() const {
  sensor::PointCloudXYZI map;
  std::lock_guard<std::mutex> lock(map_mutex_);
  const auto append_session = [&map](const std::vector<Keyframe*>& keyframes) {
    for (const Keyframe* keyframe : keyframes) {
      if (keyframe == nullptr || !keyframe->point_cloud) {
        continue;
      }
      const Eigen::Matrix3d& rotation = keyframe->inertial_state.R;
      const Eigen::Vector3d& translation = keyframe->inertial_state.p;
      for (const PointType& point : keyframe->point_cloud->points) {
        const Eigen::Vector3d world =
            rotation * Eigen::Vector3d(point.x, point.y, point.z) + translation;
        PointType output = point;
        output.x = static_cast<float>(world.x());
        output.y = static_cast<float>(world.y());
        output.z = static_cast<float>(world.z());
        map.push_back(output);
      }
    }
  };
  for (const PreviousSession& session : previous_sessions_) {
    const std::vector<Keyframe*> keyframe_view =
        MakeKeyframeView(session.keyframes);
    append_session(keyframe_view);
  }
  const std::vector<Keyframe*> current_keyframe_view =
      MakeKeyframeView(keyframes_);
  append_session(current_keyframe_view);
  return map;
}

std::vector<InertialState> TrajectoryBuilder::KeyframePoseSnapshot() const {
  std::vector<InertialState> poses;
  std::lock_guard<std::mutex> lock(map_mutex_);
  poses.reserve(scan_poses_.size());
  for (const ScanPosePtr& scan : scan_poses_) {
    if (scan) {
      poses.push_back(scan->x);
    }
  }
  return poses;
}

void TrajectoryBuilder::SetConfig(const TrajectoryBuilderOptions& config) {
  std::lock_guard<std::mutex> lock(frontend_state_mutex_);
  if (initialized_) {
    LOG(ERROR) << "TrajectoryBuilder configuration cannot change after init";
    return;
  }
  config_ = config;
}

bool TrajectoryBuilder::initialized() const {
  std::lock_guard<std::mutex> lock(frontend_state_mutex_);
  return initialized_;
}

void TrajectoryBuilder::ConfigureSlidingVoxelMap() {
  sliding_voxel_map_options_ = std::make_shared<SlidingVoxelMapOptions>();
  sliding_voxel_map_options_->voxel_size = config_.voxel_size;
  sliding_voxel_map_options_->minimum_plane_eigenvalue =
      config_.minimum_plane_eigenvalue;
  sliding_voxel_map_options_->maximum_layer = config_.maximum_voxel_layer;
  sliding_voxel_map_options_->plane_eigenvalue_thresholds =
      config_.plane_eigenvalue_thresholds;
  sliding_voxel_map_options_->plane_eigenvalue_thresholds.resize(
      static_cast<std::size_t>(config_.maximum_voxel_layer + 1), 1.0);
  sliding_voxel_map_options_->minimum_point_count_by_layer.resize(
      static_cast<std::size_t>(config_.maximum_voxel_layer + 1), 5.0);
  for (double& threshold :
       sliding_voxel_map_options_->plane_eigenvalue_thresholds) {
    if (threshold > 1e-9) {
      threshold = 1.0 / threshold;
    }
  }
  sliding_voxel_map_options_->sliding_window_order.resize(
      static_cast<std::size_t>(config_.local_window_size));
  for (int window_index = 0; window_index < config_.local_window_size;
       ++window_index) {
    sliding_voxel_map_options_
        ->sliding_window_order[static_cast<std::size_t>(window_index)] =
        window_index;
  }
}

void TrajectoryBuilder::Initialize() {
  std::lock_guard<std::mutex> frontend_lock(frontend_state_mutex_);
  if (initialized_) {
    LOG(ERROR) << "Voxel-SLAM odometry cannot be initialized more than once";
    return;
  }
  if (config_.local_window_size <= 0 || config_.worker_thread_count <= 0) {
    initialized_ = false;
    LOG(ERROR) << "Invalid Voxel-SLAM configuration: win_size and "
                  "worker_thread_count must be positive";
    return;
  }
  trajectory_lifecycle_.Stop();
  trajectory_lifecycle_.Start();
  ConfigureSlidingVoxelMap();
  lidar_imu_extrinsic_state_.R = config_.lidar_to_imu_rotation;
  lidar_imu_extrinsic_state_.p = config_.lidar_to_imu_translation;
  inertial_state_filter_.lid_rot_to_imu = config_.lidar_to_imu_rotation;
  inertial_state_filter_.lid_offset_to_imu = config_.lidar_to_imu_translation;
  inertial_state_filter_.cov_gyr = config_.odometry_gyroscope_covariance;
  inertial_state_filter_.cov_acc = config_.odometry_accelerometer_covariance;
  inertial_state_filter_.point_notime = config_.point_time_unavailable ? 1 : 0;
  inertial_state_filter_.livox_imu_scale = config_.livox_imu_scale_enabled;

  inertial_state_window_.assign(static_cast<size_t>(config_.local_window_size),
                                InertialState());
  point_variance_window_.assign(static_cast<size_t>(config_.local_window_size),
                                nullptr);
  sliding_window_partitions_.clear();
  sliding_window_partitions_.resize(
      static_cast<size_t>(std::max(1, config_.worker_thread_count)));
  voxel_operation_thread_pool_ = std::make_unique<common::ThreadPool>(
      static_cast<std::size_t>(config_.worker_thread_count));
  sliding_voxel_map_operations_ = std::make_unique<SlidingVoxelMapOperations>(
      voxel_operation_thread_pool_.get(),
      static_cast<std::size_t>(config_.worker_thread_count));
  sliding_window_state_count_ = 0;
  motion_initialization_required_ = 1;
  motion_initialization_retry_countdown_ = 0;
  motion_initialization_input_ = MotionInitializationInput{};
  motion_initialization_input_.lidar_to_imu_extrinsic =
      lidar_imu_extrinsic_state_;
  motion_initialization_input_.range_noise_meters = config_.range_noise_meters;
  motion_initialization_input_.beam_angle_noise_degrees =
      config_.beam_angle_noise_degrees;
  motion_initialization_input_.point_time_unavailable =
      config_.point_time_unavailable;
  current_inertial_state_.SetZero();
  motion_initialization_pose_anchor_ = current_inertial_state_;

  if (config_.loop_closure_enabled && config_.triangle_descriptors_enabled) {
    triangle_loop_detector_ =
        std::make_unique<loop_closure::triangle::TriangleLoopDetector>(
            config_.triangle);
  }
  if (config_.global_bundle_adjustment_enabled) {
    global_mapping_.Start([this] { return HasReadyGbaWindow(); },
                          [this] { RunGlobalMappingWork(); });
  }
  LoadPreviousSessions();
  EnsureSessionTrackingState();
  initialized_ = true;
  LOG(INFO) << "TrajectoryBuilder initialized (win="
            << config_.local_window_size << " voxel=" << config_.voxel_size
            << ")";
}

void TrajectoryBuilder::LoadPreviousSessions() {
  if (config_.previous_sessions.empty() || config_.storage_directory.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(map_mutex_);
  PreviousSessionOptions options;
  options.base_path = config_.storage_directory;
  options.session_list = config_.previous_sessions;
  options.window_size = config_.local_window_size;
  options.voxel_size = config_.voxel_size;
  options.loop_closure = config_.triangle;
  if (!PreviousSessionLoader().Load(options, &previous_sessions_,
                                    &loop_closure_edges_)) {
    LOG(WARNING) << "PreviousSessionLoader failed to load configured sessions";
    return;
  }
  history_kfsize_ = 0;
  for (const PreviousSession& session : previous_sessions_) {
    history_kfsize_ += static_cast<int>(session.keyframes.size());
  }
  cur_session_id_ = static_cast<int>(previous_sessions_.size());
  EnsureSessionTrackingState();
  ++map_state_version_;
  LOG(INFO) << "Loaded " << previous_sessions_.size()
            << " previous sessions, history_kf=" << history_kfsize_
            << " cur_session=" << cur_session_id_;
}

void TrajectoryBuilder::EnsureSessionTrackingState() {
  const int n = cur_session_id_ + 1;
  if (static_cast<int>(traveled_distance_by_session_meters_.size()) < n) {
    traveled_distance_by_session_meters_.resize(static_cast<size_t>(n), 0.0);
  }
  if (static_cast<int>(loop_closure_counts_by_session_.size()) < n) {
    const int fill = config_.historical_session_update_interval;
    loop_closure_counts_by_session_.resize(static_cast<size_t>(n), fill);
  }
}

void TrajectoryBuilder::AddLidar(const sensor::PointCloudXYZI::Ptr& cloud,
                                 double header_time) {
  if (!cloud) {
    return;
  }
  sensor::PointCloudXYZI filtered;
  ProcessLidarVendor(*cloud, &filtered, config_.lidar);
  auto ptr = std::make_shared<sensor::PointCloudXYZI>(std::move(filtered));
  sync_.PushLidar(ptr, header_time);
}

void TrajectoryBuilder::AddImu(const sensor::ImuData& imu) {
  sync_.PushImu(imu);
}

bool TrajectoryBuilder::LioStateEstimation(
    SharedPointObservationList point_observations) {
  if (!point_observations || point_observations->empty()) {
    return false;
  }
  const InertialState x_prop = current_inertial_state_;
  constexpr int kMaxIter = 4;
  Eigen::Matrix<double, kInertialStateDimension, kInertialStateDimension> G =
      Eigen::Matrix<double, kInertialStateDimension,
                    kInertialStateDimension>::Zero();
  Eigen::Matrix<double, kInertialStateDimension, kInertialStateDimension>
      H_T_H = Eigen::Matrix<double, kInertialStateDimension,
                            kInertialStateDimension>::Zero();
  const Eigen::Matrix<double, kInertialStateDimension, kInertialStateDimension>
      I_STATE = Eigen::Matrix<double, kInertialStateDimension,
                              kInertialStateDimension>::Identity();
  int rematch_num = 0;
  Eigen::Matrix3d nnt = Eigen::Matrix3d::Zero();
  const Eigen::Matrix<double, kInertialStateDimension, kInertialStateDimension>
      cov_inv = current_inertial_state_.cov.inverse();
  const int psize = static_cast<int>(point_observations->size());
  std::vector<SlidingVoxelTree*> octos(static_cast<size_t>(psize), nullptr);
  int match_num = 0;
  int residual_count = 0;
  double squared_residual_sum = 0.0;
  double maximum_absolute_residual = 0.0;

  for (int iterCount = 0; iterCount < kMaxIter; ++iterCount) {
    Eigen::Matrix<double, 6, 6> HTH = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> HTz = Eigen::Matrix<double, 6, 1>::Zero();
    const Eigen::Matrix3d rot_var =
        current_inertial_state_.cov.block<3, 3>(0, 0);
    const Eigen::Matrix3d tsl_var =
        current_inertial_state_.cov.block<3, 3>(3, 3);
    match_num = 0;
    residual_count = 0;
    squared_residual_sum = 0.0;
    maximum_absolute_residual = 0.0;
    nnt.setZero();

    for (int i = 0; i < psize; ++i) {
      PointObservation& point_observation =
          (*point_observations)[static_cast<size_t>(i)];
      const Eigen::Matrix3d phat =
          math::SkewSymmetric(point_observation.position);
      Eigen::Matrix3d var_world = current_inertial_state_.R *
                                      point_observation.covariance *
                                      current_inertial_state_.R.transpose() +
                                  phat * rot_var * phat.transpose() + tsl_var;
      Eigen::Vector3d wld =
          current_inertial_state_.R * point_observation.position +
          current_inertial_state_.p;
      double sigma_d = 0;
      FittedPlane* pla = nullptr;
      int flag = 0;
      if (octos[static_cast<size_t>(i)] != nullptr &&
          octos[static_cast<size_t>(i)]->inside(wld)) {
        double max_prob = 0;
        flag = octos[static_cast<size_t>(i)]->Match(
            wld, pla, max_prob, var_world, sigma_d,
            octos[static_cast<size_t>(i)]);
      } else {
        flag = Match(sliding_voxel_map_, wld, pla, var_world, sigma_d,
                     octos[static_cast<size_t>(i)], sliding_voxel_map_options_);
      }
      if (flag && pla != nullptr) {
        const double R_inv = 1.0 / (0.0005 + sigma_d);
        const double resi = pla->normal.dot(wld - pla->center);
        squared_residual_sum += resi * resi;
        maximum_absolute_residual =
            std::max(maximum_absolute_residual, std::fabs(resi));
        ++residual_count;
        Eigen::Matrix<double, 6, 1> jac;
        jac.head(3) =
            phat * current_inertial_state_.R.transpose() * pla->normal;
        jac.tail(3) = pla->normal;
        HTH += R_inv * jac * jac.transpose();
        HTz -= R_inv * jac * resi;
        nnt += pla->normal * pla->normal.transpose();
        ++match_num;
      }
    }

    H_T_H.block<6, 6>(0, 0) = HTH;
    const Eigen::Matrix<double, kInertialStateDimension,
                        kInertialStateDimension>
        K_1 = (H_T_H + cov_inv).inverse();
    G.block<kInertialStateDimension, 6>(0, 0) =
        K_1.block<kInertialStateDimension, 6>(0, 0) * HTH;
    const Eigen::Matrix<double, kInertialStateDimension, 1> vec =
        x_prop - current_inertial_state_;
    Eigen::Matrix<double, kInertialStateDimension, 1> solution =
        K_1.block<kInertialStateDimension, 6>(0, 0) * HTz + vec -
        G.block<kInertialStateDimension, 6>(0, 0) * vec.block<6, 1>(0, 0);
    current_inertial_state_ += solution;
    const Eigen::Vector3d rot_add = solution.block<3, 1>(0, 0);
    const Eigen::Vector3d tra_add = solution.block<3, 1>(3, 0);

    bool flg_converged =
        (rot_add.norm() * 57.3 < 0.01) && (tra_add.norm() * 100 < 0.015);
    if (flg_converged || ((rematch_num == 0) && (iterCount == kMaxIter - 2))) {
      ++rematch_num;
    }
    if (rematch_num >= 2 || iterCount == kMaxIter - 1) {
      current_inertial_state_.cov = (I_STATE - G) * current_inertial_state_.cov;
      break;
    }
    (void)match_num;
  }

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
  const Eigen::Matrix3d rotation_difference =
      x_prop.R.transpose() * current_inertial_state_.R;
  const Eigen::Vector3d rotation_correction = math::Log(rotation_difference);
  const Eigen::Vector3d translation_correction =
      current_inertial_state_.p - x_prop.p;
  VLOG(1) << "LiDAR constraint: matches=" << match_num
          << " residual_rms="
          << (residual_count > 0
                  ? std::sqrt(squared_residual_sum / residual_count)
                  : 0.0)
          << " residual_max=" << maximum_absolute_residual
          << " rotation_correction=" << rotation_correction.norm()
          << " translation_correction=" << translation_correction.norm()
          << " normal_eigenvalues=" << saes.eigenvalues().transpose();
  if (saes.eigenvalues()[0] <
      config_.minimum_odometry_normal_eigenvalue) {
    return false;
  }
  return true;
}

void TrajectoryBuilder::LocalBaIfReady() {
  if (!config_.local_bundle_adjustment_enabled ||
      sliding_window_state_count_ < config_.local_window_size) {
    return;
  }
  PointCloudFactor voxhess(config_.local_window_size);
  const std::vector<common::PoseState> pose_window =
      MakePoseWindow(inertial_state_window_);
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    sliding_voxel_map_operations_->Recut(
        &sliding_voxel_map_, sliding_window_state_count_, &pose_window,
        &voxhess, &sliding_window_partitions_);
    sliding_window_pool_ = std::move(sliding_window_partitions_[0]);
  }
  if (voxhess.point_clusters_by_window.empty()) {
    return;
  }
  Eigen::MatrixXd hess;
  std::deque<InertialPreintegrator*> preintegrator_view;
  for (const auto& preintegrator : imu_preintegrators_) {
    preintegrator_view.push_back(preintegrator.get());
  }
  const InertialState latest_state_before_optimization =
      inertial_state_window_[static_cast<std::size_t>(
          config_.local_window_size - 1)];
  // Voxel-SLAM: g_update==2 forces Gravity LI-BA after cross-session loop.
  if (gravity_update_state_ == 2 || config_.gravity_bundle_adjustment_enabled) {
    GravityConstrainedBundleAdjuster opt(config_.inertial_factor_coefficient,
                                         voxel_operation_thread_pool_.get(),
                                         config_.worker_thread_count);
    std::vector<double> resis;
    opt.Optimize(inertial_state_window_, voxhess, preintegrator_view, resis,
                 &hess, 5);
    if (gravity_update_state_ == 2) {
      gravity_update_state_ = 0;
      LOG(INFO) << "g_update Gravity LI-BA g.norm="
                << inertial_state_window_[0].g.norm();
    }
  } else {
    InertialBundleAdjuster opt(config_.inertial_factor_coefficient,
                               voxel_operation_thread_pool_.get(),
                               config_.worker_thread_count);
    opt.Optimize(inertial_state_window_, voxhess, preintegrator_view, &hess);
  }
  current_inertial_state_ = inertial_state_window_[static_cast<size_t>(
      config_.local_window_size - 1)];
  const Eigen::Matrix3d bundle_adjustment_rotation_difference =
      latest_state_before_optimization.R.transpose() *
      current_inertial_state_.R;
  const Eigen::Vector3d bundle_adjustment_rotation_correction =
      math::Log(bundle_adjustment_rotation_difference);
  const Eigen::Vector3d bundle_adjustment_translation_correction =
      current_inertial_state_.p - latest_state_before_optimization.p;
  VLOG(1) << "Local BA latest-pose correction: rotation="
          << bundle_adjustment_rotation_correction.norm()
          << " translation="
          << bundle_adjustment_translation_correction.norm()
          << " factors=" << voxhess.point_clusters_by_window.size();
  // ScanPose odom edge noise from Hessian (Voxel-SLAM bl->v6).
  if (hess.rows() >= kInertialStateDimension + 6 &&
      hess.cols() >= kInertialStateDimension + 6) {
    last_scan_v6_ = hess.block<6, 6>(0, kInertialStateDimension).diagonal();
    for (int i = 0; i < 6; ++i) {
      last_scan_v6_[i] = 1.0 / std::max(1e-12, std::fabs(last_scan_v6_[i]));
    }
  }
}

bool TrajectoryBuilder::TrySessionLoops(
    const sensor::PointCloudXYZI& source_plane_cloud,
    const std::vector<loop_closure::triangle::TriangleDescriptor>&
        triangle_descriptors,
    int query_keyframe_id, const InertialState& query_state,
    double session_traveled_distance_meters) {
  if (triangle_descriptors.empty() || source_plane_cloud.empty()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(map_mutex_);
  EnsureSessionTrackingState();
  std::vector<int> connected_sessions;
  loop_closure_edges_.Connect(cur_session_id_, &connected_sessions);
  if (connected_sessions.empty()) {
    connected_sessions.push_back(cur_session_id_);
  }
  bool should_optimize = false;
  for (std::size_t session_index = 0; session_index < previous_sessions_.size();
       ++session_index) {
    const PreviousSession& session = previous_sessions_[session_index];
    should_optimize |= TrySessionLoop(
        static_cast<int>(session_index), session.loop_detector.get(),
        session.descriptor_score_threshold, source_plane_cloud,
        triangle_descriptors, query_keyframe_id, query_state,
        session_traveled_distance_meters, &connected_sessions);
  }
  should_optimize |=
      TrySessionLoop(cur_session_id_, triangle_loop_detector_.get(),
                     config_.descriptor_score_threshold, source_plane_cloud,
                     triangle_descriptors, query_keyframe_id, query_state,
                     session_traveled_distance_meters, &connected_sessions);
  for (int& it : loop_closure_counts_by_session_) {
    ++it;
  }
  return should_optimize;
}

bool TrajectoryBuilder::TrySessionLoop(
    int session_id, TriangleLoopDetector* detector, double score_threshold,
    const sensor::PointCloudXYZI& source_plane_cloud,
    const std::vector<TriangleDescriptor>& triangle_descriptors,
    int query_keyframe_id, const InertialState& query_state,
    double session_traveled_distance_meters,
    std::vector<int>* connected_sessions) {
  if (detector == nullptr || connected_sessions == nullptr) {
    return false;
  }
  LoopSearchResult candidate;
  if (!VerifyLoopCandidate(detector, source_plane_cloud, triangle_descriptors,
                           query_keyframe_id, score_threshold, &candidate)) {
    return false;
  }
  InertialState match_pose;
  if (!FindLoopMatchPose(session_id, candidate.match_frame_id, &match_pose)) {
    return false;
  }
  const double drift_meters =
      (match_pose.R * candidate.transform.first + match_pose.p - query_state.p)
          .norm();
  bool should_optimize = false;
  const bool accepted =
      session_id == cur_session_id_
          ? AcceptSameSessionLoop(
                candidate.match_frame_id, candidate.score, drift_meters,
                session_traveled_distance_meters, &should_optimize)
          : AcceptCrossSessionLoop(session_id, candidate.match_frame_id,
                                   candidate.score, drift_meters,
                                   connected_sessions, &should_optimize);
  if (!accepted) {
    return false;
  }
  Eigen::Matrix<double, 6, 1> information =
      Eigen::Matrix<double, 6, 1>::Constant(1e-4);
  loop_closure_edges_.Push(
      session_id, cur_session_id_, candidate.match_frame_id, query_keyframe_id,
      candidate.transform.second, candidate.transform.first, information);
  ++map_state_version_;
  return should_optimize;
}

bool TrajectoryBuilder::VerifyLoopCandidate(
    TriangleLoopDetector* detector,
    const sensor::PointCloudXYZI& source_plane_cloud,
    const std::vector<TriangleDescriptor>& triangle_descriptors,
    int query_keyframe_id, double score_threshold,
    LoopSearchResult* candidate) const {
  if (candidate == nullptr) {
    return false;
  }
  *candidate = detector->SearchLoop(triangle_descriptors, source_plane_cloud,
                                    query_keyframe_id);
  if (!candidate->ok() || candidate->score <= score_threshold) {
    return false;
  }
  const sensor::PointCloudXYZI* target_plane =
      detector->PlaneCloud(candidate->match_db_index);
  return target_plane != nullptr && !target_plane->empty() &&
         NormalIcp(source_plane_cloud, *target_plane, &candidate->transform,
                   config_.loop_icp_minimum_eigenvalue);
}

bool TrajectoryBuilder::FindLoopMatchPose(int session_id, int match_keyframe_id,
                                          InertialState* match_pose) const {
  if (match_pose == nullptr) {
    return false;
  }
  const std::vector<ScanPosePtr>* scans = nullptr;
  if (session_id == cur_session_id_) {
    scans = &scan_poses_;
  } else if (session_id >= 0 &&
             session_id < static_cast<int>(previous_sessions_.size())) {
    scans = &previous_sessions_[static_cast<std::size_t>(session_id)].scans;
  }
  if (scans != nullptr && match_keyframe_id >= 0 &&
      match_keyframe_id < static_cast<int>(scans->size()) &&
      (*scans)[static_cast<std::size_t>(match_keyframe_id)]) {
    *match_pose = (*scans)[static_cast<std::size_t>(match_keyframe_id)]->x;
    return true;
  }
  for (const auto& keyframe_owner : keyframes_) {
    const Keyframe* keyframe = keyframe_owner.get();
    if (keyframe != nullptr && keyframe->keyframe_id == match_keyframe_id) {
      *match_pose = keyframe->inertial_state;
      return true;
    }
  }
  return false;
}

double TrajectoryBuilder::FindKeyframeDistance(int keyframe_id) const {
  for (const auto& keyframe_owner : keyframes_) {
    const Keyframe* keyframe = keyframe_owner.get();
    if (keyframe != nullptr && keyframe->keyframe_id == keyframe_id) {
      return keyframe->trajectory_distance_meters;
    }
  }
  return 0.0;
}

bool TrajectoryBuilder::AcceptSameSessionLoop(
    int match_keyframe_id, double score, double drift_meters,
    double session_traveled_distance_meters, bool* should_optimize) {
  const double match_distance = FindKeyframeDistance(match_keyframe_id);
  const double span =
      std::max(1e-3, session_traveled_distance_meters - match_distance);
  LOG(INFO) << "Loop same-session id=" << match_keyframe_id
            << " score=" << score << " drift=" << drift_meters
            << " span=" << span;
  const bool accepted = drift_meters / span < config_.maximum_drift_ratio;
  if (accepted &&
      loop_closure_counts_by_session_[static_cast<std::size_t>(
          cur_session_id_)] > config_.same_session_update_interval &&
      drift_meters > 0.10) {
    *should_optimize = true;
    for (int& count : loop_closure_counts_by_session_) {
      count = 0;
    }
  }
  return accepted;
}

bool TrajectoryBuilder::AcceptCrossSessionLoop(
    int session_id, int match_keyframe_id, double score, double drift_meters,
    std::vector<int>* connected_sessions, bool* should_optimize) {
  const bool already_connected =
      std::find(connected_sessions->begin(), connected_sessions->end(),
                session_id) != connected_sessions->end();
  const double session_distance =
      session_id < static_cast<int>(traveled_distance_by_session_meters_.size())
          ? traveled_distance_by_session_meters_[static_cast<std::size_t>(
                session_id)]
          : 1e-6;
  LOG(INFO) << "Loop cross-session s=" << session_id
            << " match=" << match_keyframe_id << " score=" << score
            << " drift=" << drift_meters << " jour=" << session_distance;
  if (!already_connected) {
    *should_optimize = true;
    loop_closure_counts_by_session_[static_cast<std::size_t>(session_id)] = 0;
    gravity_update_state_ = 1;
    traveled_distance_by_session_meters_[static_cast<std::size_t>(session_id)] =
        0.0;
    connected_sessions->push_back(session_id);
    return true;
  }
  if (drift_meters / std::max(session_distance, 1e-9) >= 0.05) {
    return false;
  }
  traveled_distance_by_session_meters_[static_cast<std::size_t>(session_id)] =
      1e-6;
  if (loop_closure_counts_by_session_[static_cast<std::size_t>(session_id)] >
          config_.historical_session_update_interval &&
      drift_meters > 0.25) {
    *should_optimize = true;
    for (int& count : loop_closure_counts_by_session_) {
      count = 0;
    }
  }
  return true;
}

void TrajectoryBuilder::RunOnlineLoopIncrementalOptimization() {
  PoseGraphSnapshot snapshot = BuildPoseGraphSnapshot();
  if (snapshot.empty() || snapshot.scan_poses.back().empty() ||
      !OptimizeOnlinePoseGraphSnapshot(&snapshot)) {
    return;
  }
  CommitOnlinePoseGraphSnapshot(snapshot);
}

bool TrajectoryBuilder::OptimizeOnlinePoseGraphSnapshot(
    PoseGraphSnapshot* snapshot) const {
  if (snapshot == nullptr) {
    return false;
  }
  // Online isOpt: lp edges only (no GBA), same as build_graph(..., 1).
  return OptimizePoseGraph(
      &snapshot->scan_views, &snapshot->keyframe_view_pointers,
      snapshot->loop_edges, snapshot->first_bundle_adjustment_edges,
      snapshot->second_bundle_adjustment_edges, snapshot->session_id,
      /*inject_gba=*/false);
}

void TrajectoryBuilder::CommitOnlinePoseGraphSnapshot(
    const PoseGraphSnapshot& snapshot) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (snapshot.source_pose_version != pose_correction_version_ ||
      snapshot.source_map_state_version != map_state_version_ ||
      snapshot.session_id != cur_session_id_ ||
      snapshot.scan_poses.size() != previous_sessions_.size() + 1 ||
      snapshot.keyframes.size() != previous_sessions_.size() + 1) {
    LOG(INFO) << "Discarding stale online pose graph result";
    return;
  }
  std::vector<std::vector<ScanPosePtr>*> current_scans;
  std::vector<std::vector<Keyframe*>> current_keyframe_views;
  std::vector<std::vector<Keyframe*>*> current_keyframes;
  current_scans.reserve(previous_sessions_.size() + 1);
  current_keyframe_views.reserve(previous_sessions_.size() + 1);
  current_keyframes.reserve(previous_sessions_.size() + 1);
  for (PreviousSession& session : previous_sessions_) {
    current_scans.push_back(&session.scans);
    current_keyframe_views.push_back(MakeKeyframeView(session.keyframes));
    current_keyframes.push_back(&current_keyframe_views.back());
  }
  current_scans.push_back(&scan_poses_);
  current_keyframe_views.push_back(MakeKeyframeView(keyframes_));
  current_keyframes.push_back(&current_keyframe_views.back());
  if (!PoseGraphCommitter().Commit(snapshot.scan_views,
                                   snapshot.keyframe_view_pointers,
                                   current_scans, current_keyframes)) {
    return;
  }
  ++pose_correction_version_;
  ++map_state_version_;
  ApplyOnlinePoseCorrection(snapshot.previous_current_pose);
}

void TrajectoryBuilder::ApplyOnlinePoseCorrection(
    const InertialState& previous_pose) {
  const InertialState corrected_pose = scan_poses_.back()->x;
  InertialState correction;
  correction.SetZero();
  correction.p = corrected_pose.p - corrected_pose.R *
                                        previous_pose.R.transpose() *
                                        previous_pose.p;
  correction.R = corrected_pose.R * previous_pose.R.transpose();
  const auto transform_state = [&correction](InertialState* state) {
    state->p = correction.R * state->p + correction.p;
    state->R = correction.R * state->R;
    state->v = correction.R * state->v;
  };
  if (last_keyframe_state_initialized_) {
    transform_state(&last_keyframe_state_);
  }
  for (InertialState& state : keyframe_window_poses_) {
    transform_state(&state);
  }

  for (int index = 0; index < sliding_window_state_count_; ++index) {
    InertialState& state =
        inertial_state_window_[static_cast<std::size_t>(index)];
    state.v = correction.R * state.v;
    state.p = correction.R * state.p + correction.p;
    state.R = correction.R * state.R;
    if (gravity_update_state_ == 1) {
      state.g = correction.R * state.g;
    }
  }
  if (sliding_window_state_count_ > 0) {
    const InertialState& latest_state =
        inertial_state_window_[static_cast<std::size_t>(
            sliding_window_state_count_ - 1)];
    current_inertial_state_.R = latest_state.R;
    current_inertial_state_.p = latest_state.p;
    current_inertial_state_.g = latest_state.g;
  } else {
    current_inertial_state_.R = corrected_pose.R;
    current_inertial_state_.p = corrected_pose.p;
  }
  current_inertial_state_.v = correction.R * current_inertial_state_.v;
  const double correction_distance = correction.p.norm();
  RebuildMapAfterOnlineCorrection();
  LOG(INFO) << "Online ISAM2 applied dx.p.norm=" << correction_distance
            << " history_kf=" << history_kfsize_;
}

void TrajectoryBuilder::RebuildMapAfterOnlineCorrection() {
  sliding_voxel_map_operations_->RebuildAfterPoseCorrection(
      &sliding_voxel_map_, MakePoseWindow(inertial_state_window_),
      point_variance_window_, sliding_window_state_count_,
      config_.local_window_size, &sliding_window_pool_,
      sliding_voxel_map_options_);
  const int keyframe_count = static_cast<int>(keyframes_.size());
  const int first_recent_keyframe = std::max(0, keyframe_count - 5);
  for (int index = first_recent_keyframe; index < keyframe_count; ++index) {
    Keyframe* keyframe = keyframes_[static_cast<std::size_t>(index)].get();
    if (keyframe == nullptr || keyframe->point_cloud == nullptr) {
      continue;
    }
    keyframe->map_insertion_pending = false;
    PointObservationList point_observations;
    point_observations.reserve(keyframe->point_cloud->size());
    PointObservation point_observation;
    point_observation.covariance.setZero();
    for (const PointType& point : keyframe->point_cloud->points) {
      point_observation.position << point.x, point.y, point.z;
      point_observation.position =
          keyframe->inertial_state.R * point_observation.position +
          keyframe->inertial_state.p;
      point_observation.covariance(0, 0) = point.normal_x;
      point_observation.covariance(1, 1) = point.normal_y;
      point_observation.covariance(2, 2) = point.normal_z;
      point_observations.push_back(point_observation);
    }
    CutVoxel(sliding_voxel_map_, point_observations, config_.local_window_size,
             current_session_distance_meters_, sliding_voxel_map_options_);
  }
  RestoreKeyframeMapState(keyframe_count);
  current_session_distance_meters_ = 0.0;
  if (gravity_update_state_ == 1) {
    gravity_update_state_ = 2;
  }
}

void TrajectoryBuilder::RestoreKeyframeMapState(int keyframe_count) {
  history_kfsize_ = 0;
  const int first_historical_keyframe = std::max(0, keyframe_count - 5);
  for (int index = 0; index < first_historical_keyframe; ++index) {
    if (keyframes_[static_cast<std::size_t>(index)] != nullptr) {
      keyframes_[static_cast<std::size_t>(index)]->map_insertion_pending = true;
    }
  }
  history_kfsize_ = first_historical_keyframe;
  for (PreviousSession& session : previous_sessions_) {
    for (const auto& keyframe_owner : session.keyframes) {
      if (keyframe_owner != nullptr) {
        keyframe_owner->map_insertion_pending = true;
        ++history_kfsize_;
      }
    }
  }
}

void TrajectoryBuilder::RecordScanPose(const sensor::PointCloudXYZI& cloud) {
  auto scan_pose = std::make_shared<ScanPose>(current_inertial_state_, nullptr);
  scan_pose->v6 = last_scan_v6_;
  if (!last_keyframe_state_initialized_) {
    last_keyframe_state_ = current_inertial_state_;
    last_keyframe_state_initialized_ = true;
  }

  Eigen::Isometry3f lidar_to_imu = Eigen::Isometry3f::Identity();
  lidar_to_imu.linear() = lidar_imu_extrinsic_state_.R.cast<float>();
  lidar_to_imu.translation() = lidar_imu_extrinsic_state_.p.cast<float>();
  auto body_cloud = std::make_shared<const sensor::PointCloudXYZI>(
      sensor::TransformPointCloud(cloud, lidar_to_imu));
  keyframe_window_poses_.push_back(current_inertial_state_);
  keyframe_window_clouds_.push_back(body_cloud);
  if (!config_.storage_directory.empty()) {
    session_scan_clouds_.push_back(std::move(body_cloud));
  }
  std::lock_guard<std::mutex> lock(map_mutex_);
  scan_poses_.push_back(std::move(scan_pose));
  ++map_state_version_;
}

bool TrajectoryBuilder::KeyframeWindowReady() {
  if (keyframe_window_poses_.size() <
      static_cast<std::size_t>(config_.local_window_size)) {
    return false;
  }
  if (keyframes_.empty()) {
    return true;
  }
  const double rotation_degrees =
      math::Log((last_keyframe_state_.R.transpose() *
                 current_inertial_state_.R).eval())
          .norm() *
      (180.0 / 3.14159265358979323846);
  const double translation_meters =
      (current_inertial_state_.p - last_keyframe_state_.p).norm();
  if (rotation_degrees >= 5.0 || translation_meters >= 0.1) {
    return true;
  }
  keyframe_window_poses_.pop_front();
  keyframe_window_clouds_.pop_front();
  return false;
}

sensor::PointCloudXYZI TrajectoryBuilder::BuildKeyframeCloud() const {
  sensor::PointCloudXYZI cloud;
  std::size_t point_count = 0;
  for (const auto& frame_cloud : keyframe_window_clouds_) {
    point_count += frame_cloud->size();
  }
  cloud.reserve(point_count);
  for (std::size_t index = 0; index < keyframe_window_clouds_.size(); ++index) {
    const InertialState& frame_pose = keyframe_window_poses_[index];
    const Eigen::Matrix3d rotation =
        current_inertial_state_.R.transpose() * frame_pose.R;
    const Eigen::Vector3d translation =
        current_inertial_state_.R.transpose() *
        (frame_pose.p - current_inertial_state_.p);
    for (const PointType& source_point : keyframe_window_clouds_[index]->points) {
      PointType point = source_point;
      const Eigen::Vector3d transformed =
          rotation * Eigen::Vector3d(point.x, point.y, point.z) + translation;
      point.x = static_cast<float>(transformed.x());
      point.y = static_cast<float>(transformed.y());
      point.z = static_cast<float>(transformed.z());
      cloud.push_back(point);
    }
  }
  return cloud;
}

void TrajectoryBuilder::SubmitLoopKeyframe(
    const sensor::PointCloudXYZI& cloud) {
  EnsureSessionTrackingState();
  RecordScanPose(cloud);
  if (!KeyframeWindowReady()) {
    return;
  }
  const double keyframe_distance =
      (current_inertial_state_.p - last_keyframe_state_.p).norm();
  for (double& traveled_distance : traveled_distance_by_session_meters_) {
    traveled_distance += keyframe_distance;
  }
  session_jour_ = traveled_distance_by_session_meters_[static_cast<size_t>(
      cur_session_id_)];
  const int scan_index = static_cast<int>(scan_poses_.size() - 1);
  sensor::PointCloudXYZI keyframe_cloud = BuildKeyframeCloud();
  std::vector<loop_closure::triangle::TriangleDescriptor>
      triangle_descriptors;
  sensor::PointCloudXYZI source_plane_cloud;
  if (triangle_loop_detector_) {
    triangle_loop_detector_->GenerateDescriptors(
        keyframe_cloud, scan_index, &triangle_descriptors, &source_plane_cloud);
  }
  DownsampleVoxel(&keyframe_cloud, config_.voxel_size / 10.0);
  auto keyframe = std::make_unique<Keyframe>(current_inertial_state_);
  keyframe->keyframe_id = scan_index;
  keyframe->session_id = cur_session_id_;
  keyframe->map_insertion_pending = true;
  keyframe->trajectory_distance_meters = session_jour_;
  keyframe->point_cloud = std::make_shared<const sensor::PointCloudXYZI>(
      std::move(keyframe_cloud));
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    keyframes_.push_back(std::move(keyframe));
    ++map_state_version_;
  }
  last_keyframe_state_ = current_inertial_state_;
  last_keyframe_state_initialized_ = true;
  keyframe_window_poses_.clear();
  keyframe_window_clouds_.clear();
  global_mapping_.NotifyWorkAvailable();

  if (triangle_loop_detector_) {
    const bool optimized_loop =
        TrySessionLoops(source_plane_cloud, triangle_descriptors, scan_index,
                        current_inertial_state_, session_jour_);
    triangle_loop_detector_->AddDescriptors(triangle_descriptors,
                                            source_plane_cloud);
    if (optimized_loop) {
      RunOnlineLoopIncrementalOptimization();
    }
  }
}

void TrajectoryBuilder::SystemReset() {
  LOG(WARNING) << "TrajectoryBuilder system_reset (degrade)";
  trajectory_lifecycle_.MarkDegraded();
  std::lock_guard<std::mutex> lock(map_mutex_);
  keyframe_window_poses_.clear();
  keyframe_window_clouds_.clear();
  last_keyframe_state_initialized_ = false;
  ClearInertialPreintegrators();
  sliding_voxel_map_.clear();
  sliding_window_state_count_ = 0;
  degradation_count_ = 0;
  motion_initialization_required_ = 1;
  motion_initialization_retry_countdown_ = 0;
  motion_initialization_input_ = MotionInitializationInput{};
  motion_initialization_input_.lidar_to_imu_extrinsic =
      lidar_imu_extrinsic_state_;
  motion_initialization_input_.range_noise_meters = config_.range_noise_meters;
  motion_initialization_input_.beam_angle_noise_degrees =
      config_.beam_angle_noise_degrees;
  motion_initialization_input_.point_time_unavailable =
      config_.point_time_unavailable;
  inertial_state_filter_.init_flag = false;
  inertial_state_filter_.init_num = 0;
  system_ready_ = false;
  kd_tree_odometry_.Clear();
  current_inertial_state_.ResetMotionPreservingPose();
  motion_initialization_pose_anchor_ = current_inertial_state_;
}

bool TrajectoryBuilder::RunTopDownPgo() {
  PoseGraphSnapshot snapshot = BuildPoseGraphSnapshot();
  if (snapshot.empty() || !OptimizePoseGraphSnapshot(&snapshot)) {
    return false;
  }
  return CommitPoseGraphSnapshot(snapshot);
}

PoseGraphSnapshot TrajectoryBuilder::BuildPoseGraphSnapshot() const {
  std::lock_guard<std::mutex> lock(map_mutex_);
  PoseGraphSnapshot snapshot;
  if (keyframes_.empty() && scan_poses_.empty()) {
    return snapshot;
  }
  snapshot.source_pose_version = pose_correction_version_;
  snapshot.source_map_state_version = map_state_version_;
  snapshot.session_id = cur_session_id_;
  snapshot.loop_edges = loop_closure_edges_;
  snapshot.first_bundle_adjustment_edges = global_mapping_state_.first_edges();
  snapshot.second_bundle_adjustment_edges =
      global_mapping_state_.second_edges();
  snapshot.scan_poses.reserve(previous_sessions_.size() + 1);
  snapshot.keyframes.reserve(previous_sessions_.size() + 1);
  const auto append_session = [&snapshot](const std::vector<ScanPosePtr>& scans,
                                          const auto& keyframes) {
    snapshot.scan_poses.emplace_back();
    auto& scan_snapshot = snapshot.scan_poses.back();
    scan_snapshot.reserve(scans.size());
    for (const ScanPosePtr& scan : scans) {
      scan_snapshot.push_back(scan ? std::make_shared<ScanPose>(*scan)
                                   : ScanPosePtr{});
    }
    snapshot.keyframes.emplace_back();
    auto& keyframe_snapshot = snapshot.keyframes.back();
    keyframe_snapshot.reserve(keyframes.size());
    for (const auto& keyframe : keyframes) {
      keyframe_snapshot.push_back(
          keyframe ? std::make_unique<Keyframe>(*keyframe) : nullptr);
    }
  };
  for (const PreviousSession& session : previous_sessions_) {
    append_session(session.scans, session.keyframes);
  }
  append_session(scan_poses_, keyframes_);
  snapshot.has_previous_current_pose =
      !scan_poses_.empty() && static_cast<bool>(scan_poses_.back());
  if (snapshot.has_previous_current_pose) {
    snapshot.previous_current_pose = scan_poses_.back()->x;
  }
  snapshot.BuildViews();
  return snapshot;
}

bool TrajectoryBuilder::OptimizePoseGraphSnapshot(
    PoseGraphSnapshot* snapshot) const {
  if (snapshot == nullptr) {
    return false;
  }
  return OptimizeTopDownPoseGraph(
      &snapshot->scan_views, &snapshot->keyframe_view_pointers,
      snapshot->loop_edges, snapshot->first_bundle_adjustment_edges,
      snapshot->second_bundle_adjustment_edges, snapshot->session_id);
}

bool TrajectoryBuilder::CommitPoseGraphSnapshot(
    const PoseGraphSnapshot& snapshot) {
  std::lock_guard<std::mutex> frontend_lock(frontend_state_mutex_);
  std::unique_lock<std::mutex> map_lock(map_mutex_);
  if (snapshot.source_pose_version != pose_correction_version_ ||
      snapshot.source_map_state_version != map_state_version_ ||
      snapshot.session_id != cur_session_id_ ||
      snapshot.scan_poses.size() != previous_sessions_.size() + 1 ||
      snapshot.keyframes.size() != previous_sessions_.size() + 1) {
    LOG(INFO) << "Discarding stale top-down PGO result";
    return false;
  }
  std::vector<std::vector<ScanPosePtr>*> current_scans;
  std::vector<std::vector<Keyframe*>> current_keyframe_views;
  std::vector<std::vector<Keyframe*>*> current_keyframes;
  current_scans.reserve(previous_sessions_.size() + 1);
  current_keyframe_views.reserve(previous_sessions_.size() + 1);
  current_keyframes.reserve(previous_sessions_.size() + 1);
  for (PreviousSession& session : previous_sessions_) {
    current_scans.push_back(&session.scans);
    current_keyframe_views.push_back(MakeKeyframeView(session.keyframes));
    current_keyframes.push_back(&current_keyframe_views.back());
  }
  current_scans.push_back(&scan_poses_);
  current_keyframe_views.push_back(MakeKeyframeView(keyframes_));
  current_keyframes.push_back(&current_keyframe_views.back());
  if (!PoseGraphCommitter().Commit(snapshot.scan_views,
                                   snapshot.keyframe_view_pointers,
                                   current_scans, current_keyframes)) {
    return false;
  }
  ApplyCurrentPoseCorrection(snapshot);
  ApplyLoopUpdateToMapLocked();
  map_lock.unlock();
  SyncStateMirror();
  return true;
}

void TrajectoryBuilder::ApplyCurrentPoseCorrection(
    const PoseGraphSnapshot& snapshot) {
  if (!snapshot.has_previous_current_pose || scan_poses_.empty() ||
      !scan_poses_.back()) {
    return;
  }
  const InertialState& corrected = scan_poses_.back()->x;
  const Eigen::Matrix3d delta_rotation =
      corrected.R * snapshot.previous_current_pose.R.transpose();
  const Eigen::Vector3d delta_translation =
      corrected.p - delta_rotation * snapshot.previous_current_pose.p;
  for (int index = 0; index < sliding_window_state_count_; ++index) {
    InertialState& state =
        inertial_state_window_[static_cast<std::size_t>(index)];
    state.R = delta_rotation * state.R;
    state.p = delta_rotation * state.p + delta_translation;
    state.v = delta_rotation * state.v;
  }
  current_inertial_state_.R = delta_rotation * current_inertial_state_.R;
  current_inertial_state_.p =
      delta_rotation * current_inertial_state_.p + delta_translation;
  current_inertial_state_.v = delta_rotation * current_inertial_state_.v;
  const auto transform_state = [&delta_rotation, &delta_translation](
                                   InertialState* state) {
    state->p = delta_rotation * state->p + delta_translation;
    state->R = delta_rotation * state->R;
    state->v = delta_rotation * state->v;
  };
  if (last_keyframe_state_initialized_) {
    transform_state(&last_keyframe_state_);
  }
  for (InertialState& state : keyframe_window_poses_) {
    transform_state(&state);
  }
}

void TrajectoryBuilder::RunGlobalMappingWork() {
  const auto local_configuration = MakeLocalMappingConfiguration();
  const auto global_configuration = MakeGlobalMappingConfiguration();
  GlobalMappingOptimizer optimizer(local_configuration, global_configuration);

  while (true) {
    global_mapping_.SetStage(GlobalMappingStage::kIdle);
    if (!global_mapping_.WaitForWork([this] { return HasReadyGbaWindow(); })) {
      break;
    }
    const bool finish_requested = global_mapping_.finish_requested();
    if (global_mapping_.stop_requested() && !finish_requested) {
      break;
    }
    global_mapping_.SetStage(GlobalMappingStage::kLocalWindowOptimization);
    if (!ProcessLocalMappingWindow(finish_requested, local_configuration,
                                   optimizer)) {
      continue;
    }
    if (!RunFinalGlobalMapping(global_configuration, optimizer)) {
      continue;
    }
  }
}

GlobalBundleAdjustmentConfiguration
TrajectoryBuilder::MakeLocalMappingConfiguration() const {
  GlobalBundleAdjustmentConfiguration configuration;
  configuration.voxel_size = config_.voxel_size;
  configuration.minimum_eigenvalue = config_.minimum_plane_eigenvalue;
  configuration.eigenvalue_thresholds = config_.plane_eigenvalue_thresholds;
  configuration.maximum_octree_layer = config_.maximum_voxel_layer;
  configuration.worker_thread_count = config_.worker_thread_count;
  return configuration;
}

GlobalBundleAdjustmentConfiguration
TrajectoryBuilder::MakeGlobalMappingConfiguration() const {
  GlobalBundleAdjustmentConfiguration configuration;
  configuration.voxel_size = config_.global_mapping_voxel_size;
  configuration.minimum_eigenvalue =
      config_.global_mapping_minimum_plane_eigenvalue;
  configuration.eigenvalue_thresholds =
      config_.global_mapping_eigenvalue_thresholds;
  configuration.maximum_octree_layer = config_.maximum_voxel_layer;
  configuration.maximum_iteration_count =
      config_.global_mapping_maximum_iteration_count;
  configuration.worker_thread_count = config_.worker_thread_count;
  return configuration;
}

bool TrajectoryBuilder::ProcessLocalMappingWindow(
    bool finish_requested,
    const GlobalBundleAdjustmentConfiguration& configuration,
    GlobalMappingOptimizer& optimizer) {
  GlobalMappingRequest request;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    request.request_id = global_mapping_.NextRequestId();
    request.source_pose_version = pose_correction_version_;
    request.source_map_state_version = map_state_version_;
    request.session_id = cur_session_id_;
    request.maximum_iteration_count = 1;
    request.worker_thread_count =
        std::min(2, configuration.worker_thread_count);
    request.accumulated_edges = global_mapping_state_.first_edges();
    const std::size_t next_keyframe_index =
        global_mapping_state_.next_keyframe_index();
    if (next_keyframe_index > keyframes_.size()) {
      throw std::logic_error(
          "global mapping keyframe cursor exceeds trajectory size");
    }
    const std::size_t pending_keyframe_count =
        keyframes_.size() - next_keyframe_index;
    const std::size_t minimum_window_size = static_cast<std::size_t>(
        std::max(2, config_.global_mapping_local_window_size));
    if (pending_keyframe_count < minimum_window_size && !finish_requested) {
      return false;
    }
    request = GlobalMappingRequestBuilder().BuildLocalWindowRequest(
        std::move(request), next_keyframe_index, minimum_window_size,
        keyframes_);
  }

  if (!request.keyframes.empty()) {
    std::vector<int> session_ids = {request.session_id};
    GlobalMappingResult result =
        optimizer.OptimizeLocalWindow(request, session_ids);
    if (result.status != GlobalMappingStatus::kSucceeded) {
      throw std::runtime_error(result.diagnostic);
    }
    auto submap = std::make_unique<Keyframe>(result.optimized_poses.front());
    submap->keyframe_id = request.keyframes.front().keyframe_id;
    submap->session_id = request.session_id;
    sensor::PointCloudXYZI submap_cloud = CreateSubmapPointCloud(
        request.keyframes, result.optimized_poses, configuration.voxel_size);
    submap->point_cloud =
        std::make_shared<const sensor::PointCloudXYZI>(std::move(submap_cloud));
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!global_mapping_state_.CommitLocalWindow(
            request, &result, &pose_correction_version_, &map_state_version_,
            cur_session_id_, keyframes_, std::move(submap),
            static_cast<std::size_t>(
                std::max(1, config_.global_mapping_submap_merge_size)))) {
      return false;
    }
  }

  if (!finish_requested) {
    return false;
  }
  std::lock_guard<std::mutex> lock(map_mutex_);
  return global_mapping_state_.next_keyframe_index() >= keyframes_.size();
}

bool TrajectoryBuilder::RunFinalGlobalMapping(
    const GlobalBundleAdjustmentConfiguration& configuration,
    GlobalMappingOptimizer& optimizer) {
  if (!RunSubmapOptimization(configuration, optimizer)) {
    return false;
  }
  // Final HBA operates on the merged local-window submaps, matching the
  // reference pipeline's global mapping stage.
  global_mapping_.SetStage(GlobalMappingStage::kPoseGraphOptimization);
  const auto pose_graph_start = std::chrono::steady_clock::now();
  if (!RunTopDownPgo()) {
    throw std::runtime_error("GTSAM pose graph optimization failed");
  }
  LOG(INFO) << "Global pose graph optimization completed in "
            << std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                             pose_graph_start)
                   .count()
            << " seconds";
  global_mapping_.CompleteFinishRequest();
  LOG(INFO) << "GBA finish: edges1="
            << global_mapping_state_.first_edges().edges.size()
            << " edges2=" << global_mapping_state_.second_edges().edges.size()
            << " submaps=" << global_mapping_state_.submaps().size();
  return true;
}

bool TrajectoryBuilder::RunSubmapOptimization(
    const GlobalBundleAdjustmentConfiguration& configuration,
    GlobalMappingOptimizer& optimizer) {
  global_mapping_.SetStage(GlobalMappingStage::kSubmapOptimization);
  GlobalMappingRequest request;
  std::vector<int> connected_session_ids;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    request.request_id = global_mapping_.NextRequestId();
    request.source_pose_version = pose_correction_version_;
    request.source_map_state_version = map_state_version_;
    request.session_id = cur_session_id_;
    request.maximum_iteration_count = configuration.maximum_iteration_count;
    request.worker_thread_count = configuration.worker_thread_count;
    connected_session_ids.reserve(previous_sessions_.size() + 1);
    connected_session_ids.push_back(cur_session_id_);
    for (std::size_t session_index = 0;
         session_index < previous_sessions_.size(); ++session_index) {
      connected_session_ids.push_back(static_cast<int>(session_index));
    }
    request = GlobalMappingRequestBuilder().BuildSubmapRequest(
        std::move(request), global_mapping_state_.submaps());
  }
  if (request.keyframes.size() >= 2) {
    const auto optimization_start = std::chrono::steady_clock::now();
    GlobalMappingResult result =
        optimizer.OptimizeSubmaps(request, connected_session_ids);
    LOG(INFO) << "Global submap HBA completed in "
              << std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - optimization_start)
                     .count()
              << " seconds across " << request.keyframes.size()
              << " submaps";
    if (result.status != GlobalMappingStatus::kSucceeded) {
      throw std::runtime_error(result.diagnostic);
    }
    std::lock_guard<std::mutex> lock(map_mutex_);
    return global_mapping_state_.CommitSubmapOptimization(
        request, &result, &pose_correction_version_, &map_state_version_,
        cur_session_id_);
  }
  std::lock_guard<std::mutex> lock(map_mutex_);
  return global_mapping_state_.ClearSecondEdges(
      request, pose_correction_version_, &map_state_version_, cur_session_id_);
}

void TrajectoryBuilder::RequestGlobalMappingFinish() {
  global_mapping_.RequestFinish();
}

bool TrajectoryBuilder::HasReadyGbaWindow() const {
  std::lock_guard<std::mutex> lock(map_mutex_);
  return global_mapping_state_.next_keyframe_index() <= keyframes_.size() &&
         keyframes_.size() - global_mapping_state_.next_keyframe_index() >=
             static_cast<std::size_t>(
                 std::max(2, config_.global_mapping_local_window_size));
}

void TrajectoryBuilder::ApplyLoopUpdate() {
  std::lock_guard<std::mutex> frontend_lock(frontend_state_mutex_);
  ApplyLoopUpdateLocked();
}

void TrajectoryBuilder::ApplyLoopUpdateLocked() {
  std::lock_guard<std::mutex> lock(map_mutex_);
  ApplyLoopUpdateToMapLocked();
}

void TrajectoryBuilder::ApplyLoopUpdateToMapLocked() {
  ++pose_correction_version_;
  ++map_state_version_;
  if (sliding_voxel_map_operations_ != nullptr) {
    sliding_voxel_map_operations_->RebuildAfterPoseCorrection(
        &sliding_voxel_map_, MakePoseWindow(inertial_state_window_),
        point_variance_window_, sliding_window_state_count_,
        config_.local_window_size, &sliding_window_pool_,
        sliding_voxel_map_options_);
  }
  current_session_distance_meters_ = 0.0;
}

InertialState TrajectoryBuilder::current_inertial_state() const {
  std::lock_guard<std::mutex> lock(frontend_state_mutex_);
  return current_inertial_state_;
}

void TrajectoryBuilder::SyncStateMirror() {
  state_mirror_.rot_end = current_inertial_state_.R;
  state_mirror_.pos_end = current_inertial_state_.p;
  state_mirror_.vel_end = current_inertial_state_.v;
  state_mirror_.bias_g = current_inertial_state_.bg;
  state_mirror_.bias_a = current_inertial_state_.ba;
  state_mirror_.gravity = current_inertial_state_.g;
}

InertialPreintegrationOptions TrajectoryBuilder::MakePreintegrationOptions()
    const {
  return ::atlas::frontend::inertial::MakeInertialPreintegrationOptions(
      inertial_state_filter_.scale_gravity,
      config_.local_mapping_gyroscope_covariance,
      config_.local_mapping_accelerometer_covariance,
      config_.gyroscope_bias_random_walk,
      config_.accelerometer_bias_random_walk);
}

sensor::StatesGroup TrajectoryBuilder::state() const {
  std::lock_guard<std::mutex> lock(frontend_state_mutex_);
  return state_mirror_;
}

bool TrajectoryBuilder::TryInitializeMotion(
    ::atlas::odometry::OdometryOutput* out) {
  if (static_cast<int>(motion_initialization_input_.lidar_scans.size()) <
      config_.local_window_size) {
    out->status = ::atlas::odometry::ProcessStatus::kPropagateOnly;
    out->soft_skip = true;
    SyncStateMirror();
    return true;
  }
  if (motion_initialization_retry_countdown_ > 0) {
    SlideMotionInitializationWindow();
    --motion_initialization_retry_countdown_;
    out->status = ::atlas::odometry::ProcessStatus::kPropagateOnly;
    out->soft_skip = true;
    SyncStateMirror();
    return true;
  }
  PointCloudFactor voxhess(config_.local_window_size);
  Eigen::MatrixXd hess;
  SlidingVoxelMapView slide;
  MotionInitializationWorkspace workspace{hess,
                                          voxhess,
                                          inertial_state_window_,
                                          sliding_voxel_map_,
                                          slide,
                                          *sliding_voxel_map_operations_,
                                          point_variance_window_,
                                          config_.local_window_size,
                                          sliding_window_partitions_,
                                          imu_preintegrators_,
                                          sliding_voxel_map_options_,
                                          config_.inertial_factor_coefficient,
                                          inertial_state_filter_.scale_gravity,
                                          voxel_operation_thread_pool_.get(),
                                          config_.worker_thread_count,
                                          InertialPreintegrationOptions{}};
  workspace.minimum_plane_observability_eigenvalue =
      config_.minimum_initialization_observability_eigenvalue;
  workspace.preintegration_options = MakePreintegrationOptions();
  const MotionInitializationResult initialization_result =
      MotionInitializer().Run(motion_initialization_input_, workspace);
  if (initialization_result.succeeded()) {
    motion_initialization_input_.lidar_scans.clear();
    motion_initialization_input_.imu_samples_by_scan.clear();
    motion_initialization_input_.scan_start_times.clear();
    motion_initialization_retry_countdown_ = 0;
    current_inertial_state_ = initialization_result.estimated_state;
    motion_initialization_required_ = 0;
    system_ready_ = true;
    trajectory_lifecycle_.InitializationSucceeded();
    sliding_window_state_count_ = config_.local_window_size;
    LOG(INFO) << "Motion initialization succeeded after "
              << initialization_result.iteration_count << " iterations";
    VLOG(1) << "Motion initialization state: position="
            << current_inertial_state_.p.transpose()
            << " velocity=" << current_inertial_state_.v.transpose()
            << " gravity=" << current_inertial_state_.g.transpose();
  } else {
    trajectory_lifecycle_.MarkDegraded();
    SlideMotionInitializationWindow();
    motion_initialization_retry_countdown_ =
        std::max(1, config_.local_window_size / 2);
    LOG(WARNING) << "Motion initialization failed with status "
                 << static_cast<int>(initialization_result.status) << " after "
                 << initialization_result.iteration_count << " iterations";
  }
  out->status = ::atlas::odometry::ProcessStatus::kPropagateOnly;
  out->soft_skip = true;
  SyncStateMirror();
  return true;
}

void TrajectoryBuilder::SlideMotionInitializationWindow() {
  if (!DiscardOldestMotionInitializationScan(&motion_initialization_input_)) {
    SystemReset();
    return;
  }
  const int last_window_index = config_.local_window_size - 1;
  for (int window_index = 0; window_index < last_window_index; ++window_index) {
    const std::size_t current_index = static_cast<std::size_t>(window_index);
    inertial_state_window_[current_index] =
        inertial_state_window_[current_index + 1];
    point_variance_window_[current_index] =
        point_variance_window_[current_index + 1];
  }
  inertial_state_window_[static_cast<std::size_t>(last_window_index)] =
      current_inertial_state_;
  point_variance_window_[static_cast<std::size_t>(last_window_index)] =
      nullptr;
  if (!imu_preintegrators_.empty()) {
    imu_preintegrators_.pop_front();
  }
  motion_initialization_pose_anchor_.g = current_inertial_state_.g;
  const double latest_timestamp = current_inertial_state_.t;
  RebaseMotionInitializationWindow(&inertial_state_window_,
                                   motion_initialization_pose_anchor_);
  current_inertial_state_ = motion_initialization_pose_anchor_;
  current_inertial_state_.t = latest_timestamp;
}

bool TrajectoryBuilder::Process(::atlas::odometry::OdometryOutput* out) {
  std::lock_guard<std::mutex> frontend_lock(frontend_state_mutex_);
  if (!initialized_ || finalizing_ || out == nullptr) {
    return false;
  }
  sync_.SetSlamMode(sensor::kOnlyLio);
  sensor::LidarMeasureGroup meas;
  if (!sync_.Sync(meas)) {
    out->status = ::atlas::odometry::ProcessStatus::kNoPackage;
    return false;
  }
  if (!meas.lidar || meas.lidar->empty()) {
    out->status = ::atlas::odometry::ProcessStatus::kEmptyLidar;
    out->soft_skip = true;
    return true;
  }

  const sensor::PointCloudXYZI raw_lidar_scan = *meas.lidar;
  sensor::PointCloudXYZI pl = raw_lidar_scan;

  std::deque<sensor::ImuData> imus;
  for (const auto& mg : meas.measures) {
    imus.insert(imus.end(), mg.imu.begin(), mg.imu.end());
  }
  inertial_state_filter_.pcl_beg_time = meas.lidar_frame_beg_time;
  inertial_state_filter_.pcl_end_time = meas.lidar_frame_end_time > 0
                                            ? meas.lidar_frame_end_time
                                            : meas.lidar_frame_beg_time;
  const int ekf_ret =
      inertial_state_filter_.Process(&current_inertial_state_, &pl, &imus);
  if (ekf_ret == 0) {
    out->status = ::atlas::odometry::ProcessStatus::kPropagateOnly;
    out->soft_skip = true;
    SyncStateMirror();
    return true;
  }
  const sensor::PointCloudXYZI undistorted_scan = pl;

  if (motion_initialization_required_) {
    sensor::PointCloudXYZI initialization_odometry_scan = undistorted_scan;
    DownsampleVoxel(&initialization_odometry_scan,
                    std::max(0.5, config_.point_downsampling_size_meters));
    auto initialization_observations =
        std::make_shared<PointObservationList>();
    PointCovarianceEstimator::InitializePointObservations(
        lidar_imu_extrinsic_state_, initialization_odometry_scan,
        config_.range_noise_meters, config_.beam_angle_noise_degrees,
        initialization_observations.get());
    if (kd_tree_odometry_.AddPointsOrCheckReadiness(
            *initialization_observations, current_inertial_state_)) {
      kd_tree_odometry_.EstimatePose(initialization_observations,
                                     &current_inertial_state_);
    }

    sensor::PointCloudXYZI initialization_scan = raw_lidar_scan;
    ::atlas::frontend::odometry::DownsampleInitializationScan(
        &initialization_scan, config_.point_downsampling_size_meters);
    motion_initialization_input_.lidar_scans.push_back(
        std::make_shared<sensor::PointCloudXYZI>(
            std::move(initialization_scan)));
    motion_initialization_input_.imu_samples_by_scan.push_back(imus);
    motion_initialization_input_.scan_start_times.push_back(
        meas.lidar_frame_beg_time);
    const size_t idx = motion_initialization_input_.lidar_scans.size() - 1;
    if (idx < inertial_state_window_.size()) {
      inertial_state_window_[idx] = current_inertial_state_;
    }
    if (idx > 0 && idx <= imu_preintegrators_.size() + 1) {
      auto pre = std::make_unique<InertialPreintegrator>(
          inertial_state_window_[idx - 1].bg,
          inertial_state_window_[idx - 1].ba, MakePreintegrationOptions());
      pre->PushImu(imus);
      imu_preintegrators_.push_back(std::move(pre));
    }
    const bool processed = TryInitializeMotion(out);
    if (processed) {
      SyncStateMirror();
      out->state = state_mirror_;
      out->timestamp = current_inertial_state_.t;
      out->undistorted =
          std::make_shared<sensor::PointCloudXYZI>(undistorted_scan);
    }
    return processed;
  }

  pl = undistorted_scan;
  DownsampleVoxel(&pl, config_.point_downsampling_size_meters);
  if (pl.size() < 500) {
    pl = undistorted_scan;
    DownsampleVoxel(&pl,
                    config_.point_downsampling_size_meters * 0.5);
  }

  system_ready_ = true;
  if (trajectory_lifecycle_.value() == TrajectoryLifecycleState::kDegraded) {
    trajectory_lifecycle_.InitializationSucceeded();
  }
  auto point_observations = std::make_shared<PointObservationList>();
  PointCovarianceEstimator::InitializePointObservations(
      lidar_imu_extrinsic_state_, pl, config_.range_noise_meters,
      config_.beam_angle_noise_degrees, point_observations.get());

  bool ok = true;
  if (sliding_voxel_map_.empty()) {
    // Bootstrap like Voxel-SLAM lio_state_estimation_kdtree.
    if (kd_tree_odometry_.AddPointsOrCheckReadiness(*point_observations,
                                                    current_inertial_state_)) {
      ok = kd_tree_odometry_.EstimatePose(point_observations,
                                          &current_inertial_state_);
    }
  } else {
    ok = LioStateEstimation(point_observations);
  }
  VLOG(1) << "LiDAR update: accepted=" << ok
          << " position=" << current_inertial_state_.p.transpose()
          << " velocity=" << current_inertial_state_.v.transpose()
          << " gravity=" << current_inertial_state_.g.transpose();
  degradation_count_ = UpdateDegradationCount(degradation_count_, !ok);
  if (degradation_count_ > config_.degradation_count_limit) {
    SystemReset();
    kd_tree_odometry_.Clear();
    out->status = ::atlas::odometry::ProcessStatus::kPropagateOnly;
    out->soft_skip = true;
    return true;
  }

  VectorList<3> pwld;
  TransformPointObservationsToWorld(
      common::PoseState(current_inertial_state_.t, current_inertial_state_.R,
                        current_inertial_state_.p,
                        current_inertial_state_.cov.block<6, 6>(0, 0)),
      point_observations.get(), &pwld);

  if (sliding_window_state_count_ >= config_.local_window_size) {
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      PointCloudFactor dummy(config_.local_window_size);
      const std::vector<common::PoseState> pose_window =
          MakePoseWindow(inertial_state_window_);
      sliding_voxel_map_operations_->Marginalize(
          &sliding_voxel_map_, current_session_distance_meters_,
          sliding_window_state_count_, &pose_window, &dummy,
          &sliding_window_pool_);
      sliding_voxel_map_options_->AdvanceWindowOrder(1);
    }
    if (!imu_preintegrators_.empty()) {
      imu_preintegrators_.pop_front();
    }
    for (int i = 0; i < config_.local_window_size - 1; ++i) {
      inertial_state_window_[static_cast<size_t>(i)] =
          inertial_state_window_[static_cast<size_t>(i + 1)];
      point_variance_window_[static_cast<size_t>(i)] =
          point_variance_window_[static_cast<size_t>(i + 1)];
    }
    sliding_window_state_count_ = config_.local_window_size - 1;
  }

  inertial_state_window_[static_cast<size_t>(sliding_window_state_count_)] =
      current_inertial_state_;
  point_variance_window_[static_cast<size_t>(sliding_window_state_count_)] =
      point_observations;
  auto pre = std::make_unique<InertialPreintegrator>(
      current_inertial_state_.bg, current_inertial_state_.ba,
      MakePreintegrationOptions());
  pre->PushImu(imus);
  imu_preintegrators_.push_back(std::move(pre));

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    SlidingVoxelMapView feat_tem;
    if (sliding_window_partitions_.empty()) {
      sliding_window_partitions_.resize(1);
    }
    sliding_window_partitions_[0] = std::move(sliding_window_pool_);
    CutVoxel(sliding_voxel_map_, point_observations,
             sliding_window_state_count_, feat_tem, config_.local_window_size,
             pwld, sliding_window_partitions_[0], sliding_voxel_map_options_);
    sliding_window_pool_ = std::move(sliding_window_partitions_[0]);
    const std::vector<common::PoseState> pose_window =
        MakePoseWindow(inertial_state_window_);
    for (auto& kv : feat_tem) {
      if (kv.second) {
        kv.second->Recut(sliding_window_state_count_ + 1, pose_window,
                         sliding_window_pool_);
      }
    }
  }

  ++sliding_window_state_count_;
  LocalBaIfReady();
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (history_kfsize_ > 0) {
      for (PreviousSession& session : previous_sessions_) {
        const std::vector<Keyframe*> keyframe_view =
            MakeKeyframeView(session.keyframes);
        if (KeyframeLoading(
                &sliding_voxel_map_, keyframe_view, current_inertial_state_.p,
                10.0, config_.local_window_size, &history_kfsize_,
                current_session_distance_meters_, sliding_voxel_map_options_)) {
          break;
        }
      }
    }
  }
  SubmitLoopKeyframe(pl);
  SyncStateMirror();

  out->state = state_mirror_;
  out->timestamp = current_inertial_state_.t;
  out->undistorted =
      std::make_shared<sensor::PointCloudXYZI>(undistorted_scan);
  out->map_updated = true;
  out->status = ::atlas::odometry::ProcessStatus::kOk;
  return true;
}

}  // namespace frontend
}  // namespace atlas

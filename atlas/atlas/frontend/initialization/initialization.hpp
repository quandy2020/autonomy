/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_FRONTEND_INIT_HPP_
#define ATLAS_FRONTEND_INIT_HPP_

#include <Eigen/Core>
#include <deque>
#include <memory>
#include <unordered_map>
#include <vector>

#include "common/thread_pool.hpp"
#include "frontend/inertial/inertial_preintegrator.hpp"
#include "frontend/inertial/inertial_state.hpp"
#include "map/point_cloud_factor.hpp"
#include "map/point_observation.hpp"
#include "map/sliding_voxel_map_operations.hpp"
#include "map/sliding_voxel_tree.hpp"
#include "map/voxel_key_hash.hpp"
#include "sensor/imu_data.hpp"
#include "sensor/point_types.hpp"

namespace atlas {
namespace frontend {
namespace initialization {

using ::atlas::map::PointCloudFactor;
using ::atlas::map::PointObservation;
using ::atlas::map::PointObservationList;
using ::atlas::map::SharedPointObservationList;

using ::atlas::frontend::inertial::InertialPreintegrationOptions;
using ::atlas::frontend::inertial::InertialPreintegrator;
using ::atlas::frontend::inertial::InertialState;
using ::atlas::map::FrameWindow;
using ::atlas::map::SlidingVoxelMap;
using ::atlas::map::SlidingVoxelMapOperations;
using ::atlas::map::SlidingVoxelMapOptions;
using ::atlas::map::SlidingVoxelMapView;

/** @brief Seed gravity from mean accel (simple IMU align). */
bool AlignGravity(const std::deque<sensor::ImuData>& imus, InertialState* x,
                  double scale_gravity = 1.0);

/** @brief True when enough scans are buffered for motion initialization. */
bool HasEnoughMotionInitializationScans(int buffered_scan_count,
                                        int minimum_scan_count = 5);

/** @brief Rotate window states so gravity aligns with ±z (align_gravity). */
bool AlignStateWindowWithGravity(std::vector<InertialState>* state_window);

/**
 * @brief Init-time motion undistortion into PointObservation list
 * (motion_blur).
 * @param point_notime If true, skip time-based undistortion.
 */
void UndistortInitializationScan(const sensor::PointCloudXYZI& point_cloud,
                                 PointObservationList* point_observations,
                                 InertialState current_state,
                                 const InertialState& previous_state,
                                 const std::deque<sensor::ImuData>& imu_samples,
                                 double scan_start_time,
                                 const InertialState& lidar_to_imu_extrinsic,
                                 bool point_time_unavailable,
                                 double gravity_scale = 1.0);

struct MotionInitializationInput {
  std::vector<sensor::PointCloudXYZI::Ptr> lidar_scans;
  std::vector<std::deque<sensor::ImuData>> imu_samples_by_scan;
  std::vector<double> scan_start_times;
  InertialState lidar_to_imu_extrinsic;
  double range_noise_meters = 0.05;
  double beam_angle_noise_degrees = 0.2;
  bool point_time_unavailable = false;
};

/** @brief Advances a failed initialization window by one scan. */
bool DiscardOldestMotionInitializationScan(MotionInitializationInput* input);

/** @brief Rebases a motion window so its latest state matches a world pose. */
bool RebaseMotionInitializationWindow(
    std::vector<InertialState>* state_window,
    const InertialState& pose_anchor);

enum class MotionInitializationStatus {
  kInitialized,
  kInvalidInput,
  kInsufficientScans,
  kInsufficientConstraints,
  kDegraded,
  kNotConverged,
};

struct MotionInitializationResult {
  MotionInitializationStatus status = MotionInitializationStatus::kInvalidInput;
  Eigen::Vector3d plane_observability_eigenvalues = Eigen::Vector3d::Zero();
  InertialState estimated_state;
  double gravity_magnitude = 0.0;
  int iteration_count = 0;

  bool succeeded() const {
    return status == MotionInitializationStatus::kInitialized;
  }
};

struct MotionInitializationWorkspace {
  Eigen::MatrixXd& hessian;
  PointCloudFactor& lidar_factor;
  std::vector<InertialState>& state_window;
  SlidingVoxelMap& surface_map;
  SlidingVoxelMapView& sliding_surface_map;
  SlidingVoxelMapOperations& map_operations;
  std::vector<SharedPointObservationList>& point_observation_window;
  int local_window_size;
  std::vector<std::vector<std::unique_ptr<FrameWindow>>>&
      frame_window_partitions;
  std::deque<std::unique_ptr<InertialPreintegrator>>& inertial_preintegrators;
  std::shared_ptr<const SlidingVoxelMapOptions> map_options;
  double inertial_factor_coefficient;
  double gravity_scale;
  common::ThreadPoolInterface* task_executor = nullptr;
  int worker_thread_count = 1;
  InertialPreintegrationOptions preintegration_options;
  double minimum_plane_observability_eigenvalue = 15.0;
};

/** @brief Runs motion initialization over a mutable window workspace. */
class MotionInitializer {
 public:
  MotionInitializationResult Run(
      const MotionInitializationInput& initialization_input,
      MotionInitializationWorkspace& workspace) const;
};

}  // namespace initialization
}  // namespace frontend
}  // namespace atlas

#endif

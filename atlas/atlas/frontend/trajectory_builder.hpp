/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_FRONTEND_TRAJECTORY_BUILDER_HPP_
#define ATLAS_FRONTEND_TRAJECTORY_BUILDER_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "backend/global_mapping/global_mapping.hpp"
#include "backend/global_mapping/global_mapping_state.hpp"
#include "backend/global_mapping/previous_session_loader.hpp"
#include "backend/loop_closure/triangle/triangle_loop_detector.hpp"
#include "common/thread_pool.hpp"
#include "frontend/inertial/inertial_preintegrator.hpp"
#include "frontend/inertial/inertial_state.hpp"
#include "frontend/inertial/inertial_state_filter.hpp"
#include "frontend/initialization/initialization.hpp"
#include "frontend/local_mapping/gravity_constrained_bundle_adjuster.hpp"
#include "frontend/local_mapping/inertial_bundle_adjuster.hpp"
#include "frontend/odometry/kd_tree_odometry.hpp"
#include "frontend/odometry/lidar_vendor.hpp"
#include "frontend/odometry/scan_pose.hpp"
#include "frontend/trajectory_builder_options.hpp"
#include "frontend/trajectory_builder_output.hpp"
#include "frontend/trajectory_lifecycle.hpp"
#include "map/point_cloud_factor.hpp"
#include "map/point_observation.hpp"
#include "map/sliding_voxel_map_operations.hpp"
#include "map/sliding_voxel_tree.hpp"
#include "map/voxel_key_hash.hpp"
#include "sensor/sensor_sync.hpp"

namespace atlas {
namespace backend {
namespace global_mapping {
class GlobalMappingOptimizer;
}  // namespace global_mapping
}  // namespace backend
namespace frontend {

struct PoseGraphSnapshot;

namespace loop_closure = ::atlas::backend::loop_closure;

using ::atlas::map::BundleAdjustmentKeyframe;
using ::atlas::map::FrameWindow;
using ::atlas::map::GlobalBundleAdjustmentConfiguration;
using ::atlas::map::HierarchicalBundleAdjustment;
using ::atlas::map::Keyframe;
using ::atlas::map::SlidingVoxelMap;
using ::atlas::map::SlidingVoxelMapOperations;
using ::atlas::map::SlidingVoxelMapOptions;
using ::atlas::map::SlidingVoxelMapView;
using ::atlas::map::SlidingVoxelTree;

using ::atlas::backend::global_mapping::GlobalMapping;
using ::atlas::backend::global_mapping::GlobalMappingState;
using ::atlas::backend::global_mapping::PgoEdges;
using ::atlas::backend::global_mapping::PreviousSession;
using TriangleDescriptor =
    ::atlas::backend::loop_closure::triangle::TriangleDescriptor;
using TriangleLoopDetector =
    ::atlas::backend::loop_closure::triangle::TriangleLoopDetector;
using LoopSearchResult = TriangleLoopDetector::SearchLoopResult;

using ::atlas::frontend::initialization::MotionInitializationInput;
using ::atlas::frontend::initialization::MotionInitializationResult;
using ::atlas::frontend::initialization::MotionInitializationWorkspace;
using ::atlas::frontend::initialization::MotionInitializer;
using ::atlas::frontend::initialization::RebaseMotionInitializationWindow;
using ::atlas::frontend::local_mapping::GravityConstrainedBundleAdjuster;
using ::atlas::frontend::local_mapping::InertialBundleAdjuster;
using ::atlas::frontend::odometry::KdTreeOdometry;
using ::atlas::frontend::odometry::ScanPosePtr;
using ::atlas::map::PointCloudFactor;
using ::atlas::map::SharedPointObservationList;

using ::atlas::frontend::inertial::InertialPreintegrationOptions;
using ::atlas::frontend::inertial::InertialPreintegrator;
using ::atlas::frontend::inertial::InertialState;
using ::atlas::frontend::inertial::InertialStateFilter;
using ::atlas::frontend::inertial::kInertialStateDimension;

/** @brief Voxel-SLAM pipeline: init → EKF odom → LI-BA → loop → GBA. */
class TrajectoryBuilder {
 public:
  ATLAS_SHARED_PTR_DEFINITIONS(TrajectoryBuilder);

  TrajectoryBuilder();
  explicit TrajectoryBuilder(const TrajectoryBuilderOptions& config);
  ~TrajectoryBuilder();

  void SetConfig(const TrajectoryBuilderOptions& config);
  void Initialize();
  bool initialized() const;

  void AddLidar(const sensor::PointCloudXYZI::Ptr& cloud, double header_time);
  void AddImu(const sensor::ImuData& imu);
  bool Process(::atlas::odometry::OdometryOutput* out);
  sensor::StatesGroup state() const;

  void RequestGlobalMappingFinish();
  /** @brief Drain background GBA work and persist the current session. */
  bool Finalize(std::string* error = nullptr);
  /** @brief Snapshot map keyframes in the world frame. */
  sensor::PointCloudXYZI GlobalMapSnapshot() const;
  /** @brief Snapshot the optimized poses retained for loop keyframes. */
  std::vector<InertialState> KeyframePoseSnapshot() const;

  /** @brief Rebuild local map after loop pose correction. */
  void ApplyLoopUpdate();

  InertialState current_inertial_state() const;

 private:
  TrajectoryBuilderOptions config_;
  TrajectoryLifecycle trajectory_lifecycle_;
  sensor::SensorSync sync_;
  InertialStateFilter inertial_state_filter_;
  KdTreeOdometry kd_tree_odometry_;
  InertialState current_inertial_state_;
  InertialState motion_initialization_pose_anchor_;
  InertialState lidar_imu_extrinsic_state_;
  sensor::StatesGroup state_mirror_;

  SlidingVoxelMap sliding_voxel_map_;
  std::vector<std::unique_ptr<FrameWindow>> sliding_window_pool_;
  std::vector<std::vector<std::unique_ptr<FrameWindow>>>
      sliding_window_partitions_;
  std::unique_ptr<common::ThreadPool> voxel_operation_thread_pool_;
  std::unique_ptr<SlidingVoxelMapOperations> sliding_voxel_map_operations_;
  std::vector<InertialState> inertial_state_window_;
  std::deque<std::unique_ptr<InertialPreintegrator>> imu_preintegrators_;
  std::vector<SharedPointObservationList> point_variance_window_;

  MotionInitializationInput motion_initialization_input_;
  int motion_initialization_required_ = 1;
  int motion_initialization_retry_countdown_ = 0;
  int sliding_window_state_count_ = 0;
  std::shared_ptr<SlidingVoxelMapOptions> sliding_voxel_map_options_;
  int degradation_count_ = 0;
  double current_session_distance_meters_ = 0.0;
  bool initialized_ = false;
  bool system_ready_ = false;
  bool finalizing_ = false;

  std::unique_ptr<loop_closure::triangle::TriangleLoopDetector>
      triangle_loop_detector_;
  std::vector<ScanPosePtr> scan_poses_;
  std::vector<std::unique_ptr<Keyframe>> keyframes_;
  std::deque<InertialState> keyframe_window_poses_;
  std::deque<std::shared_ptr<const sensor::PointCloudXYZI>>
      keyframe_window_clouds_;
  std::vector<std::shared_ptr<const sensor::PointCloudXYZI>>
      session_scan_clouds_;
  int history_kfsize_ = 0;
  std::vector<PreviousSession> previous_sessions_;
  PgoEdges loop_closure_edges_;
  GlobalMappingState global_mapping_state_;
  int cur_session_id_ = 0;
  double session_jour_ = 0.0;

  /** @brief Per-session counters (Voxel-SLAM relc_counts / jours). */
  std::vector<int> loop_closure_counts_by_session_;
  std::vector<double> traveled_distance_by_session_meters_;
  InertialState last_keyframe_state_;
  bool last_keyframe_state_initialized_ = false;
  /** @brief 1=rotate gravity on next loop apply; 2=force Gravity LI-BA. */
  int gravity_update_state_ = 0;
  Eigen::Matrix<double, 6, 1> last_scan_v6_ =
      Eigen::Matrix<double, 6, 1>::Constant(1e-4);

  GlobalMapping global_mapping_;
  /** @brief Serializes frontend state with asynchronous global corrections. */
  mutable std::mutex frontend_state_mutex_;
  mutable std::mutex map_mutex_;
  std::uint64_t pose_correction_version_ = 0;
  std::uint64_t map_state_version_ = 0;

  void ConfigureSlidingVoxelMap();
  void ShutdownGlobalMapping();
  void ClearInertialPreintegrators();
  bool SaveSession(std::string* error);
  void SystemReset();
  bool LioStateEstimation(SharedPointObservationList point_observations);
  void LocalBaIfReady();
  void RecordScanPose(const sensor::PointCloudXYZI& cloud);
  bool KeyframeWindowReady();
  sensor::PointCloudXYZI BuildKeyframeCloud() const;
  void SubmitLoopKeyframe(const sensor::PointCloudXYZI& cloud);
  /** @return true if online ISAM2 should run (Voxel-SLAM isOpt). */
  bool TrySessionLoops(
      const sensor::PointCloudXYZI& source_plane_cloud,
      const std::vector<loop_closure::triangle::TriangleDescriptor>&
          triangle_descriptors,
      int query_keyframe_id, const InertialState& query_state,
      double session_traveled_distance_meters);
  bool TrySessionLoop(
      int session_id, TriangleLoopDetector* detector, double score_threshold,
      const sensor::PointCloudXYZI& source_plane_cloud,
      const std::vector<TriangleDescriptor>& triangle_descriptors,
      int query_keyframe_id, const InertialState& query_state,
      double session_traveled_distance_meters,
      std::vector<int>* connected_sessions);
  bool VerifyLoopCandidate(
      TriangleLoopDetector* detector,
      const sensor::PointCloudXYZI& source_plane_cloud,
      const std::vector<TriangleDescriptor>& triangle_descriptors,
      int query_keyframe_id, double score_threshold,
      LoopSearchResult* candidate) const;
  bool FindLoopMatchPose(int session_id, int match_keyframe_id,
                         InertialState* match_pose) const;
  double FindKeyframeDistance(int keyframe_id) const;
  bool AcceptSameSessionLoop(int match_keyframe_id, double score,
                             double drift_meters,
                             double session_traveled_distance_meters,
                             bool* should_optimize);
  bool AcceptCrossSessionLoop(int session_id, int match_keyframe_id,
                              double score, double drift_meters,
                              std::vector<int>* connected_sessions,
                              bool* should_optimize);
  /** @brief Caller holds frontend_state_mutex_; commit acquires map_mutex_. */
  void RunOnlineLoopIncrementalOptimization();
  bool OptimizeOnlinePoseGraphSnapshot(PoseGraphSnapshot* snapshot) const;
  void CommitOnlinePoseGraphSnapshot(const PoseGraphSnapshot& snapshot);
  void ApplyOnlinePoseCorrection(const InertialState& previous_pose);
  void RebuildMapAfterOnlineCorrection();
  void RestoreKeyframeMapState(int keyframe_count);
  void EnsureSessionTrackingState();
  void RunGlobalMappingWork();
  GlobalBundleAdjustmentConfiguration MakeLocalMappingConfiguration() const;
  GlobalBundleAdjustmentConfiguration MakeGlobalMappingConfiguration() const;
  bool ProcessLocalMappingWindow(
      bool finish_requested,
      const GlobalBundleAdjustmentConfiguration& configuration,
      ::atlas::backend::global_mapping::GlobalMappingOptimizer& optimizer);
  bool RunFinalGlobalMapping(
      const GlobalBundleAdjustmentConfiguration& configuration,
      ::atlas::backend::global_mapping::GlobalMappingOptimizer& optimizer);
  bool RunSubmapOptimization(
      const GlobalBundleAdjustmentConfiguration& configuration,
      ::atlas::backend::global_mapping::GlobalMappingOptimizer& optimizer);
  bool HasReadyGbaWindow() const;
  bool RunTopDownPgo();
  PoseGraphSnapshot BuildPoseGraphSnapshot() const;
  bool OptimizePoseGraphSnapshot(PoseGraphSnapshot* snapshot) const;
  bool CommitPoseGraphSnapshot(const PoseGraphSnapshot& snapshot);
  void ApplyCurrentPoseCorrection(const PoseGraphSnapshot& snapshot);
  void ApplyLoopUpdateLocked();
  void ApplyLoopUpdateToMapLocked();
  void SyncStateMirror();
  bool TryInitializeMotion(::atlas::odometry::OdometryOutput* out);
  void SlideMotionInitializationWindow();
  InertialPreintegrationOptions MakePreintegrationOptions() const;
  void LoadPreviousSessions();
};

}  // namespace frontend
}  // namespace atlas

#endif  // ATLAS_FRONTEND_TRAJECTORY_BUILDER_HPP_

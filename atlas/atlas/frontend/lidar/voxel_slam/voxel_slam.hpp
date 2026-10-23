/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_VXSLAM_ODOMETRY_HPP_
#define ATLAS_VXSLAM_ODOMETRY_HPP_

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "backend/global_mapping/previous_map.hpp"
#include "backend/loop_closure/lidar/triangle_descriptor/triangle_detector.hpp"
#include "frontend/lidar/imu/imu_ekf.hpp"
#include "frontend/lidar/imu/imu_preintegration.hpp"
#include "frontend/lidar/imu/imu_state.hpp"
#include "frontend/lidar/initialization/initialization.hpp"
#include "frontend/lidar/local_mapping/li_ba_optimizer.hpp"
#include "frontend/lidar/local_mapping/lidar_factor.hpp"
#include "frontend/lidar/odometry/kdtree_odom.hpp"
#include "frontend/lidar/odometry/lidar_vendor.hpp"
#include "frontend/lidar/odometry/point_var.hpp"
#include "frontend/lidar/odometry/scan_pose.hpp"
#include "frontend/lidar/voxel_slam/odometry_interface.hpp"
#include "frontend/lidar/voxel_slam/voxel_slam_config.hpp"
#include "map/voxel/sliding_octree.hpp"
#include "map/voxel/voxel_key_hash.hpp"
#include "optimization/voxel/pose_graph.hpp"
#include "sensor/sensor_sync.hpp"

namespace atlas {
namespace vxslam {

/** @brief Voxel-SLAM pipeline: init → EKF odom → LI-BA → loop → GBA. */
class Odometry : public ::atlas::odometry::OdometryInterface {
 public:
  ATLAS_SHARED_PTR_DEFINITIONS(Odometry);

  Odometry();
  explicit Odometry(const OdometryConfig& config);
  ~Odometry() override;

  void SetConfig(const OdometryConfig& config);
  void Initialize();

  void AddLidar(const sensor::PointCloudXYZI::Ptr& cloud,
                double header_time) override;
  void AddImu(const sensor::ImuData& imu) override;
  bool Process(::atlas::odometry::OdometryOutput* out) override;
  const frontend::StatesGroup& state() const override;

  void RequestFinishGba();
  /** @brief Drain background GBA work and persist the current session. */
  bool Finalize(std::string* error = nullptr);
  /** @brief Snapshot map keyframes in the world frame. */
  sensor::PointCloudXYZI GlobalMapSnapshot() const;
  /** @brief Snapshot the optimized poses retained for loop keyframes. */
  std::vector<ImuState> KeyframePoseSnapshot() const;

  /** @brief Rebuild local map after loop pose correction. */
  void ApplyLoopUpdate();

  ImuState x_curr() const { return x_curr_; }

 private:
  OdometryConfig config_;
  frontend::SensorSync sync_;
  ImuEkf imu_ekf_;
  KdTreeOdom kd_odom_;
  ImuState x_curr_;
  ImuState extrin_;
  frontend::StatesGroup state_mirror_;

  std::unordered_map<VoxelKey, SlidingOctree*> feat_map_;
  std::vector<FrameWindow*> sw_pool_;
  std::vector<std::vector<FrameWindow*>> sws_;
  std::vector<ImuState> x_buf_;
  std::deque<ImuPreintegration*> imu_pres_;
  std::vector<PointVarListPtr> pvec_buf_;

  MotionInitInput init_buf_;
  int motion_init_flag_ = 1;
  int win_count_ = 0;
  std::vector<int> mp_;
  int degrade_cnt_ = 0;
  int keyframe_id_ = 0;
  double jour_ = 0.0;
  bool initialized_ = false;
  bool system_ready_ = false;

  std::unique_ptr<loop_closure::triangle::TriangleDetector> triangle_;
  /** @brief Gauss–Newton fallback when GTSAM ISAM2 is unavailable. */
  backend::PoseGraph pose_graph_;

  std::vector<ScanPosePtr> scan_poses_;
  std::vector<Keyframe*> keyframes_;
  int history_kfsize_ = 0;
  std::vector<PreviousMapSession> prev_sessions_;
  PgoEdges lp_edges_;
  PgoEdges gba_edges1_;
  PgoEdges gba_edges2_;

  /** @brief Bottom-up HBA local window + submap buffer. */
  std::deque<int> gba_local_ids_;
  std::vector<Keyframe*> gba_submaps_;
  int cur_session_id_ = 0;
  double session_jour_ = 0.0;

  /** @brief Per-session counters (Voxel-SLAM relc_counts / jours). */
  std::vector<int> relc_counts_;
  std::vector<double> jours_;
  ImuState x_key_;
  bool x_key_init_ = false;
  /** @brief 1=rotate gravity on next loop apply; 2=force Gravity LI-BA. */
  int g_update_ = 0;
  Eigen::Matrix<double, 6, 1> last_scan_v6_ =
      Eigen::Matrix<double, 6, 1>::Constant(1e-4);

  std::atomic<bool> finish_gba_{false};
  std::atomic<bool> stop_{false};
  std::thread gba_thread_;
  mutable std::mutex map_mutex_;
  std::vector<ImuState> gba_poses_;
  std::vector<std::shared_ptr<const sensor::PointCloudXYZI>> gba_clouds_;

  void ApplyConfigGlobals();
  void ShutdownGba();
  void ClearImuPreintegrations();
  bool SaveSession(std::string* error);
  void SystemReset();
  bool LioStateEstimation(PointVarListPtr pvec);
  void LocalBaIfReady();
  void SubmitLoopKeyframe(const sensor::PointCloudXYZI& cloud);
  /** @return true if online ISAM2 should run (Voxel-SLAM isOpt). */
  bool TrySessionLoops(
      const sensor::PointCloudXYZI& plane_src,
      const std::vector<loop_closure::triangle::TriangleDescriptor>& stds,
      int query_id, const ImuState& xc, double smp_jour);
  void RunOnlineLoopIsam2();
  void EnsureSessionCounters();
  void GbaWorker();
  void RunTopDownPgo();
  void SyncStateMirror();
  bool TryMotionInit(::atlas::odometry::OdometryOutput* out);
  void LoadPreviousMaps();
};

}  // namespace vxslam
}  // namespace atlas

#endif

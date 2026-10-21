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
 * @file argus_node.hpp
 * @brief Autolink adapter for Argus LivoOdometry (sensors in, viz out).
 */

#ifndef AUTONOMY_LOCALIZATION_ARGUS_NODE_HPP_
#define AUTONOMY_LOCALIZATION_ARGUS_NODE_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "autolink/autolink.hpp"
#include "autonomy/transform/buffer.hpp"

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>

#include "argus/map/ikd_tree.hpp"
#include "argus/map/occupancy_grid_builder.hpp"
#include "argus/odometry/livo_odometry.hpp"
#include "argus/odometry/livo_odometry_config.hpp"
#include "argus/sensor/point_types.hpp"

namespace autonomy {
namespace localization {

/**
 * @brief Runs Argus LivoOdometry from autolink topics.
 *
 * Publishes map-frame odometry, trajectory, registered / global clouds,
 * 2D occupancy grid, and TF (map→odom, optional odom→base_link).
 */
class ArgusNode {
public:
    struct Options {
        std::string config_path;
        std::string imu_topic = "/imu";
        std::string lidar_topic = "/points";
        std::string image_topic = "/image";
        std::string map_frame = "map";
        std::string odom_frame = "odom";
        std::string base_frame = "base_link";
        std::string lidar_frame;
        /** Map-frame odom (GT seed applied when ready) for Autoviz overlay. */
        std::string odometry_topic = "/argus/odometry";
        /**
         * LIO-local odom (no GT seed) — same pose offline writes to estimate.tum.
         * Use this for online ATE so Umeyama protocol matches offline.
         */
        std::string odometry_lio_topic = "/argus/odometry_lio";
        std::string trajectory_topic = "/argus/trajectory";
        std::string cloud_map_topic = "/argus/cloud_map";
        std::string cloud_registered_topic = "/argus/cloud_registered";
        std::string occupancy_topic = "/argus/occupancy";
        std::string tf_topic = "/tf";
        std::string tf_static_topic = "/tf_static";
        double map_period_sec = 2.0;
        /** Off by default: huge grids blow SHM and starve /argus/trajectory. */
        bool enable_occupancy_publish = false;
        /** Soft cap on Path poses kept/published (full Path each Write). */
        int trajectory_max_poses = 800;
        /** Min interval between /argus/trajectory SHM writes (sec). */
        double trajectory_period_sec = 0.2;
        /** Soft cap on unique voxels kept in /argus/cloud_map (never FIFO-evict). */
        int global_cloud_max_points = 5000000;
        /**
         * Downsample every Nth point from each scan before ikd insert.
         * Live default >1: ikd AddPoints every frame was starving Autolink.
         */
        int global_cloud_stride = 3;
        /** World-frame voxel leaf for ikd downsample (m); matches MapBuilder. */
        float global_cloud_leaf_m = 0.1f;
        /** Ikd rebuild when size grows by this factor since last balance. */
        double global_ikd_rebuild_ratio = 1.5;
        /**
         * Cap points in each SHM PointCloud2 publish. Autolink SHM has few
         * large blocks; huge clouds stall writers in TryLockForWrite and
         * starve all subsequent Argus logs/topics.
         */
        /** Keep PointCloud2 under ~1MB so SHM stays on the 64-block tier. */
        int registered_publish_max_points = 40000;
        /** Full-snapshot stride target (~1MB SHM-friendly). */
        int map_publish_max_points = 120000;
        /**
         * Publish /argus/cloud_map as deltas (new voxels) for Autoviz Decay
         * accumulation. Prefer false for live Autoviz: full strided snapshots
         * with decay_time=0 never "forget" early map when max_points caps FIFO.
         */
        bool map_publish_incremental = false;
        /**
         * When incremental=true, every N map publishes also send a full ikd
         * snapshot so Autoviz can heal regions dropped by max_points FIFO.
         * 1 = every map publish is full (same as incremental=false for pub).
         */
        int map_full_snapshot_every_n = 10;
        /** Min interval between /argus/cloud_registered publishes (sec). */
        double registered_period_sec = 0.5;
        /**
         * Insert into viz ikd only every N ok frames (1 = every frame).
         * Reduces Flush cost so sensor callbacks return before SHM overruns.
         */
        int global_map_insert_every_n = 2;
        /** Cap spilled IMU samples when sensor mutex is busy (drop oldest). */
        int imu_spill_max = 2000;
        float occupancy_resolution = 0.1f;
        double occupancy_min_height = 0.2;
        double occupancy_max_height = 2.5;
        double occupancy_floor_height = 0.0;
        double occupancy_lidar_height = 0.2;
        /** Skip /argus/occupancy Write when W*H exceeds this (SHM blow-up). */
        int occupancy_publish_max_cells = 1500 * 1500;
        /** Stop feeding occupancy once map span exceeds this (m). */
        double occupancy_max_extent_m = 200.0;
        /**
         * Put Argus publish (odom/traj/clouds) into the same `map` as /gt/*
         * by locking ONE rigid SE(3) after a short travel (same idea as offline
         * ATE Umeyama, but scale fixed to 1):
         *   collect pairs → Umeyama(with_scale=false) → T_map_lio.
         * GT messages are never modified; ESIKF stays local; only publish TF.
         * First-pose-only seed is not enough — both start ≈I while body axes
         * can differ by ~90° until the path has shape.
         */
        bool seed_map_frame_from_gt = true;
        /**
         * Debug: multi-pair planar Umeyama+scale on the estimate (GT untouched).
         * Prefer seed_map_frame_from_gt; leave false for honest live overlay.
         */
        bool align_to_gt = false;
        std::string gt_odom_topic = "/gt/odometry";
        /**
         * Max |t_est - t_gt| when pairing samples (sec).
         * LIO stamps scan-end (~header+0.1s); datasets stamps GT at lidar
         * header — 0.15 covers one sweep without accepting a full prior frame.
         */
        double gt_align_max_dt = 0.15;
        /** Keep recent GT samples for nearest-stamp pairing. */
        int gt_pose_buffer = 64;
        /** Min horizontal travel (m) of Argus before locking planar align. */
        double gt_align_min_span_m = 3.0;
        /** Min time-matched pairs before locking planar align. */
        int gt_align_min_pairs = 20;
        /**
         * Reject seed if planar RMSE after rigid lock exceeds this (m).
         * Prevents locking onto a diverged LIO (looks like traj "too fast").
         */
        double gt_seed_max_rmse_xy_m = 1.5;
        /**
         * Reject seed if est_span / gt_span is outside [1/r, r].
         * Catches scale blow-up before Autoviz shows one traj racing ahead.
         */
        double gt_seed_max_span_ratio = 2.0;
        /**
         * Debug-only: replace traj translation with GT (breaks traj↔cloud
         * consistency). Default off — Fast-LIVO2 keeps one estimate for both.
         */
        bool trajectory_snap_to_gt = false;
    };

    explicit ArgusNode(Options options);
    ~ArgusNode();

    ArgusNode(const ArgusNode&) = delete;
    ArgusNode& operator=(const ArgusNode&) = delete;

    bool Start();
    void Shutdown();

private:
    using ImuMsg = automsgs::msgs::sensor_msgs::Imu;
    using CloudMsg = automsgs::msgs::sensor_msgs::PointCloud2;
    using ImageMsg = automsgs::msgs::sensor_msgs::Image;
    using TfMsg = automsgs::msgs::tf2_msgs::TFMessage;

    using OdomMsg = automsgs::msgs::nav_msgs::Odometry;

    void OnImu(const std::shared_ptr<ImuMsg>& msg);
    void OnCloud(const std::shared_ptr<CloudMsg>& msg);
    void OnImage(const std::shared_ptr<ImageMsg>& msg);
    void OnGtOdom(const std::shared_ptr<OdomMsg>& msg);
    void IngestTf(const TfMsg& message, const std::string& authority);
    /**
     * @brief Under sensor mutex: only Process + snapshot into @p heavy.
     *
     * Pose / ikd / SHM PointCloud2 / TF are deferred to FlushHeavyPublishes
     * (outside the sensor mutex). Holding the mutex across those stalls the
     * 400 Hz OnImu AddImu path → SHM ring drops IMU → live Z/XY diverge.
     */
    struct HeavyPublish {
        double stamp = -1.0;
        uint64_t frame_id = 0;
        argus::frontend::StatesGroup state;
        argus::sensor::PointCloudXYZI::Ptr body;
        bool publish_pose = false;
        bool append_map = false;
        bool publish_map = false;
        bool publish_registered = false;
    };
    void DrainAndPublish(std::vector<HeavyPublish>* heavy);
    void FlushHeavyPublishes(const std::vector<HeavyPublish>& heavy);
    void EnqueueHeavyPublishes(std::vector<HeavyPublish>&& heavy);
    void FlushPendingImuUnlocked();
    void PublishWorkerLoop();
    bool NeedsGtSubscription() const;
    bool MapFrameReady() const;
    bool TrySeedMapFrameFromGt(double timestamp_sec,
                               const Eigen::Matrix4d& T_est);
    bool TryAlignToGt(double timestamp_sec, const Eigen::Matrix4d& T_est);
    bool FindNearestGtUnlocked(double timestamp_sec, Eigen::Matrix4d* T_gt,
                               double* gt_stamp) const;
    void FlushBufferedTrajectory();
    Eigen::Matrix4d AlignedPose(const Eigen::Matrix4d& T_est) const;
    argus::sensor::PointCloudXYZI AlignedWorldCloud(
        const argus::frontend::StatesGroup& state,
        const argus::sensor::PointCloudXYZI& body) const;
    void PublishPose(double timestamp_sec,
                     const argus::frontend::StatesGroup& state);
    void PublishMapOdomTf(double timestamp_sec,
                          const argus::frontend::StatesGroup& state);
    void PublishRegisteredCloud(double timestamp_sec,
                                const argus::frontend::StatesGroup& state,
                                const argus::sensor::PointCloudXYZI& body);
    void PublishMapCloud(double timestamp_sec);
    /** Encode @p cloud (XYZ + jet-by-z rgb) into PointCloud2 and Write. */
    bool WriteMapCloudMsg(double timestamp_sec,
                          const argus::sensor::PointCloudXYZI& cloud,
                          const char* mode_tag);
    void PublishOccupancy(double timestamp_sec);
    void AppendToGlobalMap(const argus::frontend::StatesGroup& state,
                           const argus::sensor::PointCloudXYZI& body);
    void UpdateOccupancy(const argus::frontend::StatesGroup& state,
                         const argus::sensor::PointCloudXYZI& body);
    void SyncGlobalMapCloud();
    void ResetGlobalIkd();

    Options options_;
    argus::odometry::LivoOdometryConfig config_;
    std::unique_ptr<argus::odometry::LivoOdometry> odom_;
    std::unique_ptr<argus::map::OccupancyGridBuilder> occupancy_;
    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Odometry>>
        odom_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Odometry>>
        odom_lio_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Path>>
        trajectory_writer_;
    std::shared_ptr<autolink::Writer<CloudMsg>> cloud_writer_;
    std::shared_ptr<autolink::Writer<CloudMsg>> registered_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::map_msgs::OccupancyGrid>>
        occ_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::tf2_msgs::TFMessage>>
        tf_writer_;
    automsgs::msgs::nav_msgs::Path trajectory_;
    /**
     * Incremental ikd global map (GT-aligned world) for /argus/cloud_map.
     * One point per voxel leaf; never time-FIFO trimmed.
     */
    std::unique_ptr<argus::map::IkdTree> global_ikd_;
    argus::sensor::PointCloudXYZI global_map_;
    bool global_map_dirty_ = false;
    /**
     * New voxels since last /argus/cloud_map publish (incremental mode).
     * Cleared after each successful delta Write (or after full snapshot).
     */
    argus::sensor::PointCloudXYZI pending_map_delta_;
    /** Next map publish is a full ikd export (startup / rewind reset). */
    bool map_publish_full_next_ = true;
    /** Serializes AddImu / AddLidar / Process only. */
    std::mutex mutex_;
    /** Serializes deferred pose/map/cloud publishes (outside sensor mutex). */
    std::mutex publish_mutex_;
    /** Async publish queue — sensor callbacks must not block on SHM/ikd. */
    std::mutex publish_queue_mutex_;
    std::condition_variable publish_cv_;
    std::deque<std::vector<HeavyPublish>> publish_queue_;
    std::thread publish_thread_;
    std::atomic<bool> publish_worker_running_{false};
    /** IMU samples that arrived while Process held mutex_ (drop oldest). */
    std::mutex imu_spill_mutex_;
    std::deque<argus::sensor::ImuData> imu_spill_;
    transform::Buffer* tf_buffer_ = nullptr;
    std::atomic<bool> running_{false};
    double last_map_pub_sec_ = -1.0;
    double last_registered_pub_sec_ = -1.0;
    double last_traj_pub_sec_ = -1.0;
    uint64_t ok_frames_ = 0;
    /** Successful /argus/cloud_map publishes (for periodic full snapshots). */
    uint64_t map_pub_count_ = 0;
    struct PublishStats {
        uint64_t traj_ok = 0;
        uint64_t traj_fail = 0;
        uint64_t map_ok = 0;
        uint64_t map_fail = 0;
        uint64_t reg_ok = 0;
        uint64_t reg_fail = 0;
        uint64_t occ_ok = 0;
        uint64_t occ_fail = 0;
        uint64_t occ_skip = 0;
    } pub_stats_;
    bool occupancy_extent_capped_ = false;
    bool use_lidar_ = true;
    bool use_camera_ = false;
    bool logged_identity_odom_tf_ = false;
    bool logged_lidar_tf_ = false;
    bool logged_global_cap_ = false;
    std::string lidar_frame_;
    /**
     * Publish transform: T_pub = Sim(T_gt_align_, scale) * T_est.
     * Seed path: rigid SE(3), scale=1. Umeyama path: planar + scale.
     */
    Eigen::Matrix4d T_gt_align_ = Eigen::Matrix4d::Identity();
    double gt_align_scale_ = 1.0;
    bool map_frame_seeded_ = false;
    bool gt_aligned_ = false;
    mutable std::mutex gt_mutex_;
    struct GtSample {
        double stamp = -1.0;
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    };
    std::deque<GtSample> gt_samples_;
    bool have_gt_ = false;
    struct AlignPair {
        double stamp = -1.0;
        Eigen::Vector3d est_xyz = Eigen::Vector3d::Zero();
        Eigen::Vector3d gt_xyz = Eigen::Vector3d::Zero();
        Eigen::Matrix4d T_est = Eigen::Matrix4d::Identity();
    };
    std::vector<AlignPair> align_pairs_;
};

}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ARGUS_NODE_HPP_

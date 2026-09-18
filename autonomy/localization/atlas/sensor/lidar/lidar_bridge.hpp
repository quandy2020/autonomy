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

#pragma once

#include "autonomy/localization/atlas/backend/lidar_loop_detector.hpp"
#include "autonomy/localization/atlas/backend/lidar_pose_graph.hpp"
#include "autonomy/localization/atlas/frontend/lidar_loc/lidar_locator.hpp"
#include "autonomy/localization/atlas/frontend/lidar_loc/pose_extrapolator.hpp"
#include "autonomy/localization/atlas/frontend/lio/imu_process.hpp"
#include "autonomy/localization/atlas/frontend/lio/sync.hpp"
#include "autonomy/localization/atlas/frontend/local_estimator.hpp"
#include "autonomy/localization/atlas/io/g2p5/g2p5.hpp"
#include "autonomy/localization/atlas/mapping/lidar_keyframe.hpp"
#include "autonomy/localization/atlas/mapping/map_incremental.hpp"
#include "autonomy/localization/atlas/sensor/imu/imu_sensor.hpp"
#include "autonomy/localization/atlas/sensor/lidar/lidar_sensor.hpp"
#include "autonomy/localization/atlas/sensor/lidar/preprocess.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "autolink/autolink.hpp"
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

namespace autonomy::localization::atlas {

class system;
class VizBridge;

/**
 * Autolink PointCloud2 → LidarSensor (preprocess + ObsModel residuals).
 * Map insert via mapping::MapIncremental::IntegrateScan after FeedWithPose.
 * Sensor-layer bridge only; does not publish a second pose stream.
 * Optional LocalEstimator provides T_wb when no vision system is attached.
 *
 * Lidar loop (optional): LidarPoseGraph Optimize → sync KF T_wb → Reset State
 * (LocalEstimator) and/or map_publisher for LIVO. Not a second SLAM; does not
 * invent vision LoopClosing edges.
 */
class LidarBridge {
public:
    //! Fired after successful lidar pose-graph Optimize + State apply.
    using LoopClosedFn = std::function<void(
        const Mat44_t& T_wb_corrected, std::uint64_t query_id,
        std::uint64_t cand_id)>;

    struct Options {
        std::string topic = "/points";
        sensor::Preprocess::Options preprocess;
        int max_points_decode = 120000;
        //! Assumed scan duration when no per-point times (for sync window only).
        double default_scan_dt = 0.1;
        //! Geometric lidar loop (default off — does not disturb VO/LIVO).
        bool use_lidar_loop = false;
        //! NDT loc against prior TiledMap (LidarLocator); soft Reset estimator.
        bool use_lidar_loc = false;
        backend::LidarLoopDetector::Options lidar_loop;
        mapping::LidarKeyframeManager::Options keyframe;
        //! Publish IVox global map as PointCloud2 (LIO /atlas/cloud_map).
        bool publish_global_cloud = true;
        std::string global_cloud_topic = "/atlas/cloud_map";
        std::string global_cloud_frame = "map";
        int global_cloud_max_points = 200000;
        double global_cloud_period_sec = 0.5;
    };

    LidarBridge(system* slam, sensor::LidarSensor* lidar, Options options,
                frontend::LocalEstimator* estimator = nullptr);
    ~LidarBridge();

    LidarBridge(const LidarBridge&) = delete;
    LidarBridge& operator=(const LidarBridge&) = delete;

    void SetLocalEstimator(frontend::LocalEstimator* estimator) {
        estimator_ = estimator;
    }
    void SetVizBridge(VizBridge* viz) { viz_ = viz; }
    void SetMapIncremental(mapping::MapIncremental* map_inc) {
        map_incremental_ = map_inc;
    }
    void SetImuSensor(sensor::ImuSensor* imu) { imu_sensor_ = imu; }
    void SetG2P5(map::G2P5* g2p5) { g2p5_ = g2p5; }
    void SetLidarLocator(frontend::LidarLocator* locator) {
        lidar_locator_ = locator;
    }
    void SetPoseExtrapolator(frontend::PoseExtrapolator* extrapolator) {
        pose_extrapolator_ = extrapolator;
    }
    void SetLoopClosedCallback(LoopClosedFn fn) {
        on_loop_closed_ = std::move(fn);
    }

    Options& options() { return options_; }
    [[nodiscard]] const Options& options() const { return options_; }

    frontend::lio::ImuProcess& imu_process() { return imu_process_; }

    bool Start(const std::shared_ptr<autolink::Node>& node);
    void Stop();

private:
    void OnCloud(
        const std::shared_ptr<automsgs::msgs::sensor_msgs::PointCloud2>& msg);
    Mat44_t CurrentTwc() const;
    //! Keyframe gate shared by lidar loop + G2P5; returns true if new KF.
    bool MaybeKeyframe(double t, const Mat44_t& Twb,
                       const std::vector<Vec3_t>& cloud_body);
    void MaybeLidarLoop(const Mat44_t& Twb,
                        const std::vector<Vec3_t>& cloud_body);
    //! Optional NDT align against prior map; may soft-Reset LocalEstimator.
    void MaybeLidarLoc(double t, const std::vector<Vec3_t>& cloud_body,
                       Mat44_t* Twb_inout);
    void PublishGlobalCloud(double timestamp_sec);

    system* slam_ = nullptr;
    sensor::LidarSensor* lidar_ = nullptr;
    frontend::LocalEstimator* estimator_ = nullptr;
    mapping::MapIncremental* map_incremental_ = nullptr;
    sensor::ImuSensor* imu_sensor_ = nullptr;
    VizBridge* viz_ = nullptr;
    map::G2P5* g2p5_ = nullptr;
    frontend::LidarLocator* lidar_locator_ = nullptr;
    frontend::PoseExtrapolator* pose_extrapolator_ = nullptr;
    LoopClosedFn on_loop_closed_;
    Options options_;
    sensor::Preprocess preprocess_;
    frontend::lio::LidarImuSync sync_;
    frontend::lio::ImuProcess imu_process_;
    mapping::LidarKeyframeManager keyframe_mgr_;
    backend::LidarLoopDetector lidar_loop_;
    backend::LidarPoseGraph pose_graph_;
    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::sensor_msgs::PointCloud2>>
        global_cloud_writer_;
    std::atomic<bool> running_{false};
    std::uint64_t scan_count_ = 0;
    bool have_last_dbg_pose_ = false;
    Mat44_t last_dbg_Twb_ = Mat44_t::Identity();
    double last_dbg_t_ = -1.0;
    double last_global_cloud_pub_t_ = -1.0;
};

}  // namespace autonomy::localization::atlas

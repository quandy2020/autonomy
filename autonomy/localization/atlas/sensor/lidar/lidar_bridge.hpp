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

#include "autonomy/localization/atlas/frontend/lio/imu_process.hpp"
#include "autonomy/localization/atlas/frontend/lio/sync.hpp"
#include "autonomy/localization/atlas/frontend/local_estimator.hpp"
#include "autonomy/localization/atlas/mapping/map_incremental.hpp"
#include "autonomy/localization/atlas/sensor/imu/imu_sensor.hpp"
#include "autonomy/localization/atlas/sensor/lidar/lidar_sensor.hpp"
#include "autonomy/localization/atlas/sensor/lidar/lightning/preprocess/preprocess.hpp"

#include <atomic>
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
 */
class LidarBridge {
public:
    struct Options {
        std::string topic = "/points";
        sensor::Preprocess::Options preprocess;
        int max_points_decode = 120000;
        //! Assumed scan duration when no per-point times (for sync window only).
        double default_scan_dt = 0.1;
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

    bool Start(const std::shared_ptr<autolink::Node>& node);
    void Stop();

private:
    void OnCloud(
        const std::shared_ptr<automsgs::msgs::sensor_msgs::PointCloud2>& msg);
    Mat44_t CurrentTwc() const;

    system* slam_ = nullptr;
    sensor::LidarSensor* lidar_ = nullptr;
    frontend::LocalEstimator* estimator_ = nullptr;
    mapping::MapIncremental* map_incremental_ = nullptr;
    sensor::ImuSensor* imu_sensor_ = nullptr;
    VizBridge* viz_ = nullptr;
    Options options_;
    sensor::Preprocess preprocess_;
    frontend::lio::LidarImuSync sync_;
    frontend::lio::ImuProcess imu_process_;
    std::shared_ptr<autolink::Node> node_;
    std::atomic<bool> running_{false};
};

}  // namespace autonomy::localization::atlas

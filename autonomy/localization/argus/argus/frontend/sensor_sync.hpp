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
 * @file sensor_sync.hpp
 * @brief Multi-sensor package sync (LiDAR / IMU / Image) for LIO / LIVO / LO /
 * VIO.
 */

#ifndef ARGUS_FRONTEND_SENSOR_SYNC_HPP_
#define ARGUS_FRONTEND_SENSOR_SYNC_HPP_

#include <condition_variable>
#include <deque>
#include <mutex>

#include "argus/common/macros.hpp"
#include "argus/frontend/types.hpp"
#include "argus/sensor/image_data.hpp"
#include "argus/sensor/imu_data.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace frontend {

/**
 * @brief Buffers raw sensor streams and packs `LidarMeasureGroup` packages.
 *
 * Port of LIVO `LIVMapper::sync_packages` (ROS-free).
 */
class SensorSync
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(SensorSync);

    /**
     * @brief Select packaging policy (`kOnlyLio` / `kLivo` / `kOnlyLo` /
     *        `kOnlyVio`).
     * @param mode Slam mode enum.
     */
    void SetSlamMode(SlamMode mode) {
        slam_mode_ = mode;
    }

    /**
     * @brief Enable LiDAR stream contribution.
     * @param enable True to accept PushLidar.
     */
    void SetLidarEnabled(bool enable) {
        enable_lidar_ = enable;
    }

    /**
     * @brief Enable IMU stream contribution.
     * @param enable True to accept PushImu.
     */
    void SetImuEnabled(bool enable) {
        enable_imu_ = enable;
    }

    /**
     * @brief Enable camera stream contribution.
     * @param enable True to accept PushImage.
     */
    void SetImageEnabled(bool enable) {
        enable_image_ = enable;
    }

    /**
     * @brief Initial exposure time hint for VIO packages (s).
     * @param t Exposure duration or offset used by LIVO sync.
     */
    void SetExposureTimeInit(double t) {
        exposure_time_init_ = t;
    }

    /**
     * @brief Enqueue a LiDAR sweep.
     * @param cloud Body-frame cloud (shared ownership).
     * @param header_time Sweep end (or header) time in seconds.
     */
    void PushLidar(const sensor::PointCloudXYZI::Ptr& cloud,
                   double header_time);

    /**
     * @brief Enqueue one IMU sample.
     * @param imu Timestamped accel / gyro.
     */
    void PushImu(const sensor::ImuData& imu);

    /**
     * @brief Enqueue one camera frame.
     * @param image Timestamped image.
     */
    void PushImage(const sensor::ImageData& image);

    /**
     * @brief Try to assemble one complete measure package.
     * @param[out] meas Filled when ready.
     * @return True when a complete package is available in `meas`.
     */
    bool Sync(LidarMeasureGroup& meas);

    /** @return Last accepted LiDAR header time (s), or -1 if none. */
    double LastLidarTimestamp() const {
        return last_timestamp_lidar_;
    }
    /** @return Last accepted IMU time (s), or -1 if none. */
    double LastImuTimestamp() const {
        return last_timestamp_imu_;
    }
    /** @return Last accepted image time (s), or -1 if none. */
    double LastImageTimestamp() const {
        return last_timestamp_image_;
    }

    /** @return Mutex guarding the buffers (for external waiters). */
    std::mutex& mutex() {
        return mtx_;
    }
    /** @return Condition variable notified on push. */
    std::condition_variable& cv() {
        return cv_;
    }

private:
    bool SyncOnlyLio(LidarMeasureGroup& meas);
    bool SyncLivo(LidarMeasureGroup& meas);
    bool SyncLivoFromWaitOrVio(LidarMeasureGroup& meas);
    bool SyncLivoFromLio(LidarMeasureGroup& meas);
    bool SyncOnlyLo(LidarMeasureGroup& meas);
    bool SyncOnlyVio(LidarMeasureGroup& meas);

    SlamMode slam_mode_ = kOnlyLio;
    bool enable_lidar_ = true;
    bool enable_imu_ = true;
    bool enable_image_ = false;
    bool lidar_pushed_ = false;
    double exposure_time_init_ = 0.0;
    double last_timestamp_lidar_ = -1.0;
    double last_timestamp_imu_ = -1.0;
    double last_timestamp_image_ = -1.0;

    std::deque<sensor::PointCloudXYZI::Ptr> lidar_buffer_;
    std::deque<double> lidar_header_time_buffer_;
    std::deque<sensor::ImuData> imu_buffer_;
    std::deque<sensor::ImageData> image_buffer_;

    std::mutex mtx_;
    std::condition_variable cv_;
};

}  // namespace frontend
}  // namespace argus

#endif  // ARGUS_FRONTEND_SENSOR_SYNC_HPP_

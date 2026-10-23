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
 * @brief LiDAR / IMU package sync for Voxel-SLAM (LIO / LO).
 */

#ifndef ATLAS_FRONTEND_SENSOR_SYNC_HPP_
#define ATLAS_FRONTEND_SENSOR_SYNC_HPP_

#include <condition_variable>
#include <deque>
#include <mutex>

#include "common/macros.hpp"
#include "sensor/imu_types.hpp"
#include "sensor/point_types.hpp"
#include "sensor/states.hpp"

namespace atlas {
namespace vxslam {
namespace frontend {

/**
 * @brief Buffers LiDAR / IMU streams and packs @ref LidarMeasureGroup packages.
 */
class SensorSync {
 public:
  ATLAS_SHARED_PTR_DEFINITIONS(SensorSync);

  /**
   * @brief Select packaging policy.
   * @param[in] mode One of @c kOnlyLio / @c kOnlyLo.
   */
  void SetSlamMode(SlamMode mode) { slam_mode_ = mode; }

  /**
   * @brief Enable or disable LiDAR stream contribution.
   * @param[in] enable True to accept @ref PushLidar; false ignores pushes.
   */
  void SetLidarEnabled(bool enable) { enable_lidar_ = enable; }

  /**
   * @brief Enable or disable IMU stream contribution.
   * @param[in] enable True to accept @ref PushImu; false ignores pushes.
   */
  void SetImuEnabled(bool enable) { enable_imu_ = enable; }

  /**
   * @brief Enqueue a LiDAR sweep.
   * @param[in] cloud       Body-frame cloud (shared ownership).
   * @param[in] header_time Sweep end (or header) time in seconds.
   */
  void PushLidar(const sensor::PointCloudXYZI::Ptr& cloud, double header_time);

  /**
   * @brief Enqueue one IMU sample.
   * @param[in] imu Timestamped accel / gyro in the IMU body frame.
   */
  void PushImu(const sensor::ImuData& imu);

  /**
   * @brief Try to assemble one complete measure package.
   * @param[out] meas Filled when a complete package is ready.
   * @return True when @p meas holds a complete package; false if more data
   *         is needed.
   */
  bool Sync(LidarMeasureGroup& meas);

  /**
   * @brief Consume the sticky loopback flag (timestamp went backwards).
   * @return True if a sensor stream restarted since the last consume.
   */
  bool ConsumeLoopback();

  /** @return Last accepted LiDAR header time, or -1. */
  double LastLidarTimestamp() const { return last_timestamp_lidar_; }

  /** @return Last accepted IMU sample time, or -1. */
  double LastImuTimestamp() const { return last_timestamp_imu_; }

  /** @return Mutex guarding the internal buffers. */
  std::mutex& mutex() { return mtx_; }

  /** @return Condition variable notified on each successful push. */
  std::condition_variable& cv() { return cv_; }

 private:
  bool SyncOnlyLio(LidarMeasureGroup& meas);
  bool SyncOnlyLo(LidarMeasureGroup& meas);

  SlamMode slam_mode_ = kOnlyLio;
  bool enable_lidar_ = true;
  bool enable_imu_ = true;
  bool lidar_pushed_ = false;
  bool loopback_ = false;
  double last_timestamp_lidar_ = -1.0;
  double last_timestamp_imu_ = -1.0;
  /** Default sweep length when points lack relative time (curvature). */
  static constexpr double kDefaultScanPeriodSec = 0.1;

  std::deque<sensor::PointCloudXYZI::Ptr> lidar_buffer_;
  std::deque<double> lidar_header_time_buffer_;
  std::deque<sensor::ImuData> imu_buffer_;

  std::mutex mtx_;
  std::condition_variable cv_;
};

}  // namespace frontend
}  // namespace vxslam
}  // namespace atlas

#endif  // ATLAS_FRONTEND_SENSOR_SYNC_HPP_

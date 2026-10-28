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

#include "sensor/sensor_sync.hpp"

#include <iostream>
#include <utility>

namespace argus {
namespace vxslam {
namespace frontend {

void SensorSync::PushLidar(const sensor::PointCloudXYZI::Ptr& cloud,
                           double header_time) {
  if (!enable_lidar_ || !cloud) {
    return;
  }
  std::lock_guard<std::mutex> lock(mtx_);
  if (last_timestamp_lidar_ >= 0.0 && header_time < last_timestamp_lidar_) {
    std::cerr << "lidar loop back, clear buffer" << std::endl;
    lidar_buffer_.clear();
    lidar_header_time_buffer_.clear();
    lidar_pushed_ = false;
    loopback_ = true;
  }
  lidar_buffer_.push_back(cloud);
  lidar_header_time_buffer_.push_back(header_time);
  last_timestamp_lidar_ = header_time;
  cv_.notify_all();
}

void SensorSync::PushImu(const sensor::ImuData& imu) {
  if (!enable_imu_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mtx_);
  if (last_timestamp_imu_ >= 0.0 && imu.time < last_timestamp_imu_) {
    std::cerr << "imu loop back, clear buffer" << std::endl;
    imu_buffer_.clear();
    lidar_pushed_ = false;
    loopback_ = true;
  }
  imu_buffer_.push_back(imu);
  last_timestamp_imu_ = imu.time;
  cv_.notify_all();
}

bool SensorSync::ConsumeLoopback() {
  std::lock_guard<std::mutex> lock(mtx_);
  const bool hit = loopback_;
  loopback_ = false;
  return hit;
}

bool SensorSync::Sync(LidarMeasureGroup& meas) {
  if (lidar_buffer_.empty() && enable_lidar_) {
    return false;
  }
  if (imu_buffer_.empty() && enable_imu_) {
    return false;
  }

  switch (slam_mode_) {
    case kOnlyLio:
      return SyncOnlyLio(meas);
    case kOnlyLo:
      return SyncOnlyLo(meas);
    default:
      return false;
  }
}

bool SensorSync::SyncOnlyLio(LidarMeasureGroup& meas) {
  if (meas.last_lio_update_time < 0.0) {
    meas.last_lio_update_time = lidar_header_time_buffer_.front();
  }
  if (!lidar_pushed_) {
    meas.lidar = lidar_buffer_.front();
    if (meas.lidar->points.size() <= 1) {
      return false;
    }
    meas.lidar_frame_beg_time = lidar_header_time_buffer_.front();
    double span_sec = meas.lidar->points.back().curvature / 1000.0;
    if (span_sec < 1e-6) {
      span_sec = kDefaultScanPeriodSec;
    }
    meas.lidar_frame_end_time = meas.lidar_frame_beg_time + span_sec;
    meas.cloud_proc_cur = meas.lidar;
    lidar_pushed_ = true;
  }

  if (enable_imu_ && last_timestamp_imu_ < meas.lidar_frame_end_time) {
    return false;
  }

  MeasureGroup m;
  m.imu.clear();
  m.lio_time = meas.lidar_frame_end_time;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    while (!imu_buffer_.empty()) {
      if (imu_buffer_.front().time > meas.lidar_frame_end_time) {
        break;
      }
      m.imu.push_back(imu_buffer_.front());
      imu_buffer_.pop_front();
    }
    lidar_buffer_.pop_front();
    lidar_header_time_buffer_.pop_front();
  }
  cv_.notify_all();

  meas.ekf_phase = kLio;
  meas.measures.clear();
  meas.measures.push_back(m);
  lidar_pushed_ = false;
  return true;
}

bool SensorSync::SyncOnlyLo(LidarMeasureGroup& meas) {
  if (!lidar_pushed_) {
    if (lidar_buffer_.empty()) {
      return false;
    }
    meas.lidar = lidar_buffer_.front();
    meas.lidar_frame_beg_time = lidar_header_time_buffer_.front();
    double span_sec = meas.lidar->points.back().curvature / 1000.0;
    if (span_sec < 1e-6) {
      span_sec = kDefaultScanPeriodSec;
    }
    meas.lidar_frame_end_time = meas.lidar_frame_beg_time + span_sec;
    lidar_pushed_ = true;
  }
  MeasureGroup m;
  m.lio_time = meas.lidar_frame_end_time;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    lidar_buffer_.pop_front();
    lidar_header_time_buffer_.pop_front();
  }
  cv_.notify_all();
  lidar_pushed_ = false;
  meas.ekf_phase = kLo;
  meas.measures.clear();
  meas.measures.push_back(m);
  return true;
}

}  // namespace frontend
}  // namespace vxslam
}  // namespace argus

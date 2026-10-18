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

#include "argus/frontend/sensor_sync.hpp"

#include <cstdio>
#include <iostream>
#include <utility>

namespace argus {
namespace frontend {

void SensorSync::PushLidar(const sensor::PointCloudXYZI::Ptr& cloud,
                           double header_time) {
    if (!enable_lidar_ || !cloud) {
        return;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    if (header_time < last_timestamp_lidar_) {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer_.clear();
        lidar_header_time_buffer_.clear();
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
    if (imu.timestamp < last_timestamp_imu_) {
        std::cerr << "imu loop back, clear buffer" << std::endl;
        imu_buffer_.clear();
    }
    imu_buffer_.push_back(imu);
    last_timestamp_imu_ = imu.timestamp;
    cv_.notify_all();
}

void SensorSync::PushImage(const sensor::ImageData& image) {
    if (!enable_image_) {
        return;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    if (image.timestamp < last_timestamp_image_) {
        std::cerr << "image loop back, clear buffer" << std::endl;
        image_buffer_.clear();
    }
    image_buffer_.push_back(image);
    last_timestamp_image_ = image.timestamp;
    cv_.notify_all();
}

bool SensorSync::Sync(LidarMeasureGroup& meas) {
    if (lidar_buffer_.empty() && enable_lidar_) {
        return false;
    }
    if (image_buffer_.empty() && enable_image_) {
        return false;
    }
    if (imu_buffer_.empty() && enable_imu_) {
        return false;
    }

    switch (slam_mode_) {
        case kOnlyLio:
            return SyncOnlyLio(meas);
        case kLivo:
            return SyncLivo(meas);
        case kOnlyLo:
            return SyncOnlyLo(meas);
        case kOnlyVio:
            return SyncOnlyVio(meas);
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
        meas.lidar_frame_end_time =
            meas.lidar_frame_beg_time +
            meas.lidar->points.back().curvature / 1000.0;
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
            if (imu_buffer_.front().timestamp > meas.lidar_frame_end_time) {
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
    meas.measures.push_back(m);
    lidar_pushed_ = false;
    return true;
}

bool SensorSync::SyncLivoFromWaitOrVio(LidarMeasureGroup& meas) {
    const double img_capture_time =
        image_buffer_.front().timestamp + exposure_time_init_;
    if (meas.last_lio_update_time < 0.0) {
        meas.last_lio_update_time = lidar_header_time_buffer_.front();
    }

    const double lid_newest_time =
        lidar_header_time_buffer_.back() +
        lidar_buffer_.back()->points.back().curvature / 1000.0;
    const double imu_newest_time = imu_buffer_.back().timestamp;

    if (img_capture_time < meas.last_lio_update_time + 0.00001) {
        image_buffer_.pop_front();
        std::cerr << "[ Data Cut ] Throw one image frame!" << std::endl;
        return false;
    }
    if (img_capture_time > lid_newest_time ||
        img_capture_time > imu_newest_time) {
        return false;
    }

    MeasureGroup m;
    m.imu.clear();
    m.lio_time = img_capture_time;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        while (!imu_buffer_.empty()) {
            if (imu_buffer_.front().timestamp > m.lio_time) {
                break;
            }
            if (imu_buffer_.front().timestamp > meas.last_lio_update_time) {
                m.imu.push_back(imu_buffer_.front());
            }
            imu_buffer_.pop_front();
        }
    }
    cv_.notify_all();

    *(meas.cloud_proc_cur) = *(meas.cloud_proc_next);
    // Keep capacity for the next sync cycle (avoid free/realloc).
    meas.cloud_proc_next->clear();

    const int lid_frame_num = static_cast<int>(lidar_buffer_.size());
    const int max_size =
        static_cast<int>(meas.cloud_proc_cur->size()) + 24000 * lid_frame_num;
    meas.cloud_proc_cur->reserve(static_cast<std::size_t>(max_size));
    meas.cloud_proc_next->reserve(static_cast<std::size_t>(max_size));

    while (!lidar_buffer_.empty()) {
        if (lidar_header_time_buffer_.front() > img_capture_time) {
            break;
        }
        auto pcl = lidar_buffer_.front()->points;
        const double frame_header_time = lidar_header_time_buffer_.front();
        const float max_offs_time_ms =
            static_cast<float>((m.lio_time - frame_header_time) * 1000.0);

        for (std::size_t i = 0; i < pcl.size(); ++i) {
            auto pt = pcl[i];
            if (pcl[i].curvature < max_offs_time_ms) {
                pt.curvature += static_cast<float>(
                    (frame_header_time - meas.last_lio_update_time) * 1000.0);
                meas.cloud_proc_cur->points.push_back(pt);
            } else {
                pt.curvature += static_cast<float>(
                    (frame_header_time - m.lio_time) * 1000.0);
                meas.cloud_proc_next->points.push_back(pt);
            }
        }
        lidar_buffer_.pop_front();
        lidar_header_time_buffer_.pop_front();
    }

    meas.measures.push_back(m);
    meas.ekf_phase = kLio;
    return true;
}

bool SensorSync::SyncLivoFromLio(LidarMeasureGroup& meas) {
    const double img_capture_time =
        image_buffer_.front().timestamp + exposure_time_init_;
    meas.ekf_phase = kVio;
    meas.measures.clear();

    MeasureGroup m;
    m.vio_time = img_capture_time;
    m.lio_time = meas.last_lio_update_time;
    m.image = image_buffer_.front().image;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        image_buffer_.pop_front();
    }
    cv_.notify_all();
    meas.measures.push_back(m);
    lidar_pushed_ = false;
    return true;
}

bool SensorSync::SyncLivo(LidarMeasureGroup& meas) {
    const EkfState last_phase = meas.ekf_phase;
    switch (last_phase) {
        case kWait:
        case kVio:
            return SyncLivoFromWaitOrVio(meas);
        case kLio:
            return SyncLivoFromLio(meas);
        default:
            return false;
    }
}

bool SensorSync::SyncOnlyLo(LidarMeasureGroup& meas) {
    if (!lidar_pushed_) {
        if (lidar_buffer_.empty()) {
            return false;
        }
        meas.lidar = lidar_buffer_.front();
        meas.lidar_frame_beg_time = lidar_header_time_buffer_.front();
        meas.lidar_frame_end_time =
            meas.lidar_frame_beg_time +
            meas.lidar->points.back().curvature / 1000.0;
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
    meas.measures.push_back(m);
    return true;
}

bool SensorSync::SyncOnlyVio(LidarMeasureGroup& meas) {
    if (image_buffer_.empty()) {
        return false;
    }
    const double img_capture_time =
        image_buffer_.front().timestamp + exposure_time_init_;
    if (enable_imu_ && last_timestamp_imu_ < img_capture_time) {
        return false;
    }

    MeasureGroup m;
    m.imu.clear();
    m.vio_time = img_capture_time;
    m.lio_time = img_capture_time;
    m.image = image_buffer_.front().image;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (enable_imu_) {
            while (!imu_buffer_.empty()) {
                if (imu_buffer_.front().timestamp > img_capture_time) {
                    break;
                }
                m.imu.push_back(imu_buffer_.front());
                imu_buffer_.pop_front();
            }
        }
        image_buffer_.pop_front();
    }
    cv_.notify_all();

    meas.measures.clear();
    meas.measures.push_back(m);
    meas.ekf_phase = kVio;
    meas.last_lio_update_time = img_capture_time;
    return true;
}

}  // namespace frontend
}  // namespace argus

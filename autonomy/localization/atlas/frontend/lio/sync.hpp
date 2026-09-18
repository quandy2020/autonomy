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

//! frontend/lio/sync — simple lidar/IMU buffer; MVP pass-through when empty.

#include "autonomy/localization/atlas/frontend/lio/measure_group.hpp"

#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>
#include <vector>

namespace autonomy::localization::atlas {
namespace frontend {
namespace lio {

class LidarImuSync {
public:
    struct Options {
        std::size_t max_imu = 4000;
        std::size_t max_lidar = 8;
        //! Require IMU covering [lidar_begin, lidar_end] with this margin (s).
        double cover_margin = 0.0;
    };

    LidarImuSync() = default;
    explicit LidarImuSync(Options options) : options_(std::move(options)) {}

    void PushImu(const sensor::ImuSample& sample) {
        std::lock_guard<std::mutex> lock(mtx_);
        imu_buf_.push_back(sample);
        while (imu_buf_.size() > options_.max_imu) {
            imu_buf_.pop_front();
        }
    }

    void PushLidar(double lidar_begin, double lidar_end,
                   std::vector<Vec3_t> points_body,
                   std::vector<double> point_time_rel = {}) {
        std::lock_guard<std::mutex> lock(mtx_);
        PendingLidar pl;
        pl.begin = lidar_begin;
        pl.end = (lidar_end > lidar_begin) ? lidar_end : lidar_begin;
        pl.points = std::move(points_body);
        pl.times = std::move(point_time_rel);
        lidar_buf_.push_back(std::move(pl));
        while (lidar_buf_.size() > options_.max_lidar) {
            lidar_buf_.pop_front();
        }
    }

    //! Pop when IMU covers the oldest scan interval; otherwise false (pass-through).
    bool TryPop(MeasureGroup* out) {
        if (!out) {
            return false;
        }
        std::lock_guard<std::mutex> lock(mtx_);
        if (lidar_buf_.empty() || imu_buf_.empty()) {
            return false;
        }
        const PendingLidar& pl = lidar_buf_.front();
        const double t0 = pl.begin - options_.cover_margin;
        const double t1 = pl.end + options_.cover_margin;
        if (imu_buf_.front().timestamp > t0 || imu_buf_.back().timestamp < t1) {
            return false;
        }
        out->lidar_begin_time = pl.begin;
        out->lidar_end_time = pl.end;
        out->points_body = std::move(lidar_buf_.front().points);
        out->point_time_rel = std::move(lidar_buf_.front().times);
        out->imu.clear();
        for (const auto& s : imu_buf_) {
            if (s.timestamp < t0) {
                continue;
            }
            if (s.timestamp > t1) {
                break;
            }
            out->imu.push_back(s);
        }
        lidar_buf_.pop_front();
        // Drop IMU older than this scan begin (keep a small overlap).
        while (!imu_buf_.empty() && imu_buf_.front().timestamp < t0 - 0.05) {
            imu_buf_.pop_front();
        }
        return !out->points_body.empty();
    }

    void Clear() {
        std::lock_guard<std::mutex> lock(mtx_);
        imu_buf_.clear();
        lidar_buf_.clear();
    }

    [[nodiscard]] bool empty() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return lidar_buf_.empty();
    }

private:
    struct PendingLidar {
        double begin = 0.0;
        double end = 0.0;
        std::vector<Vec3_t> points;
        std::vector<double> times;
    };

    Options options_;
    mutable std::mutex mtx_;
    std::deque<sensor::ImuSample> imu_buf_;
    std::deque<PendingLidar> lidar_buf_;
};

}  // namespace lio
}  // namespace frontend
}  // namespace autonomy::localization::atlas

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

//! frontend/lio/sync — lidar/IMU time sync (LIO-T1).
//! Pop only when IMU covers [lidar_begin, lidar_end]; otherwise hold / drop stale.

#include "autonomy/localization/atlas/frontend/lio/measure_group.hpp"

#include <algorithm>
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
        double cover_margin = 0.002;
        //! Drop oldest lidar if still uncovered after this wait past imu_back (s).
        double max_lidar_lag = 0.35;
        //! Keep IMU samples this far before scan begin (s) for bridging.
        double imu_keep_before = 0.05;
    };

    LidarImuSync() = default;
    explicit LidarImuSync(Options options) : options_(std::move(options)) {}

    void set_options(Options options) { options_ = std::move(options); }
    [[nodiscard]] const Options& options() const { return options_; }

    void PushImu(const sensor::ImuSample& sample) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!imu_buf_.empty() &&
            sample.timestamp <= imu_buf_.back().timestamp + 1e-9) {
            return;  // monotonic dedup
        }
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
            ++dropped_lidar_;
        }
    }

    //! Pop when IMU covers the oldest scan interval; otherwise false (hold).
    //! Stale uncovered scans are dropped (counted) so the pipeline does not stall.
    bool TryPop(MeasureGroup* out) {
        if (!out) {
            return false;
        }
        std::lock_guard<std::mutex> lock(mtx_);
        DropStaleLocked();
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
        // Bridge: prepend last IMU before window if available.
        if (last_imu_valid_ && last_imu_.timestamp < t0) {
            out->imu.push_back(last_imu_);
        }
        for (const auto& s : imu_buf_) {
            if (s.timestamp < t0) {
                continue;
            }
            if (s.timestamp > t1) {
                break;
            }
            out->imu.push_back(s);
        }
        if (!out->imu.empty()) {
            last_imu_ = out->imu.back();
            last_imu_valid_ = true;
        }
        lidar_buf_.pop_front();
        while (!imu_buf_.empty() &&
               imu_buf_.front().timestamp < t0 - options_.imu_keep_before) {
            imu_buf_.pop_front();
        }
        ++popped_;
        return !out->points_body.empty() && out->imu.size() >= 2;
    }

    void Clear() {
        std::lock_guard<std::mutex> lock(mtx_);
        imu_buf_.clear();
        lidar_buf_.clear();
        last_imu_valid_ = false;
    }

    [[nodiscard]] bool empty() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return lidar_buf_.empty();
    }

    [[nodiscard]] std::size_t lidar_queued() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return lidar_buf_.size();
    }

    [[nodiscard]] std::size_t popped_count() const { return popped_; }
    [[nodiscard]] std::size_t dropped_lidar_count() const {
        return dropped_lidar_;
    }

private:
    struct PendingLidar {
        double begin = 0.0;
        double end = 0.0;
        std::vector<Vec3_t> points;
        std::vector<double> times;
    };

    void DropStaleLocked() {
        if (lidar_buf_.empty() || imu_buf_.empty()) {
            return;
        }
        const double imu_back = imu_buf_.back().timestamp;
        while (!lidar_buf_.empty()) {
            const PendingLidar& pl = lidar_buf_.front();
            const double t1 = pl.end + options_.cover_margin;
            // IMU already past scan end but never covered begin → gap / late IMU.
            if (imu_back > t1 + options_.max_lidar_lag &&
                (imu_buf_.front().timestamp > pl.begin - options_.cover_margin)) {
                lidar_buf_.pop_front();
                ++dropped_lidar_;
                continue;
            }
            // Scan far older than newest IMU and still uncovered.
            if (pl.end + options_.max_lidar_lag < imu_back &&
                imu_buf_.back().timestamp < t1) {
                // Still waiting for future IMU — keep.
                break;
            }
            if (pl.begin + options_.max_lidar_lag < imu_back &&
                imu_buf_.front().timestamp > pl.begin - options_.cover_margin) {
                lidar_buf_.pop_front();
                ++dropped_lidar_;
                continue;
            }
            break;
        }
    }

    Options options_;
    mutable std::mutex mtx_;
    std::deque<sensor::ImuSample> imu_buf_;
    std::deque<PendingLidar> lidar_buf_;
    sensor::ImuSample last_imu_;
    bool last_imu_valid_ = false;
    std::size_t popped_ = 0;
    std::size_t dropped_lidar_ = 0;
};

}  // namespace lio
}  // namespace frontend
}  // namespace autonomy::localization::atlas

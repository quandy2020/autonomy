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

#include "autonomy/localization/atlas/estimate/lidar_residual_source.hpp"

#include <deque>
#include <mutex>
#include <utility>

namespace autonomy::localization::atlas {
namespace estimate {

//! Thread-safe timed residual buffer owned by sensor/lidar (Lightning ObsModel).
class BufferedLidarResidualSource final : public ILidarResidualSource {
public:
    struct TimedBatch {
        double timestamp = 0.0;
        LidarFactorBatch batch;
    };

    explicit BufferedLidarResidualSource(std::size_t max_batches = 64)
        : max_batches_(max_batches) {}

    void Push(double timestamp, LidarFactorBatch batch) {
        std::lock_guard<std::mutex> lock(mtx_);
        buffer_.push_back(TimedBatch{timestamp, std::move(batch)});
        while (buffer_.size() > max_batches_) {
            buffer_.pop_front();
        }
    }

    LidarFactorBatch Pull(double t0, double t1) override {
        LidarFactorBatch out;
        std::lock_guard<std::mutex> lock(mtx_);
        for (const auto& item : buffer_) {
            if (item.timestamp < t0 || item.timestamp > t1) {
                continue;
            }
            out.point_planes.insert(out.point_planes.end(),
                                    item.batch.point_planes.begin(),
                                    item.batch.point_planes.end());
            out.num_point_point += item.batch.num_point_point;
        }
        return out;
    }

    [[nodiscard]] std::size_t size() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return buffer_.size();
    }

private:
    std::size_t max_batches_;
    mutable std::mutex mtx_;
    std::deque<TimedBatch> buffer_;
};

}  // namespace estimate
}  // namespace autonomy::localization::atlas

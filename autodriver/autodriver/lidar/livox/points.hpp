/*
 * Copyright 2026 Autodriver contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file
 * @brief Livox XYZIT point + publish-frequency frame assembler.
 */

#ifndef AUTODRIVER_LIDAR_LIVOX_POINTS_HPP_
#define AUTODRIVER_LIDAR_LIVOX_POINTS_HPP_

#include <cstdint>
#include <mutex>
#include <utility>
#include <vector>

namespace autodriver {
namespace lidar {
namespace livox {

struct PointXYZIT {
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    float intensity = 0.f;
    double timestamp_ns = 0.0;
};

/**
 * @brief Accumulates points and flushes a frame after publish_interval_ns.
 */
class FrameAssembler {
public:
    explicit FrameAssembler(std::uint64_t publish_interval_ns)
        : publish_interval_ns_(publish_interval_ns == 0
                                   ? 100'000'000ULL
                                   : publish_interval_ns) {}

    void SetIntervalNs(std::uint64_t interval_ns) {
        std::lock_guard<std::mutex> lock(mutex_);
        publish_interval_ns_ =
            interval_ns == 0 ? 100'000'000ULL : interval_ns;
    }

    void Append(std::vector<PointXYZIT> points) {
        if (points.empty()) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (frame_start_ns_ == 0) {
            frame_start_ns_ =
                static_cast<std::uint64_t>(points.front().timestamp_ns);
        }
        buffer_.insert(buffer_.end(),
                       std::make_move_iterator(points.begin()),
                       std::make_move_iterator(points.end()));
    }

    /**
     * @brief If the frame window elapsed, swaps out points and resets.
     * @return true when a frame was ready.
     */
    bool TryFlush(std::uint64_t now_ns, std::vector<PointXYZIT>* out) {
        if (out == nullptr) {
            return false;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.empty()) {
            return false;
        }
        if (frame_start_ns_ == 0) {
            frame_start_ns_ = now_ns;
            return false;
        }
        if (now_ns < frame_start_ns_ + publish_interval_ns_) {
            return false;
        }
        out->swap(buffer_);
        buffer_.clear();
        frame_start_ns_ = now_ns;
        return true;
    }

    void Clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.clear();
        frame_start_ns_ = 0;
    }

private:
    std::mutex mutex_;
    std::vector<PointXYZIT> buffer_;
    std::uint64_t frame_start_ns_ = 0;
    std::uint64_t publish_interval_ns_ = 100'000'000ULL;
};

}  // namespace livox
}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_LIVOX_POINTS_HPP_

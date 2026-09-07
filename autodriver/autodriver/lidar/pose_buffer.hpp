/*
 * Copyright 2026 Autodriver contributors
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
 * @file
 * @brief Thread-safe pose buffer for MotionCompensator PoseLookup.
 */

#ifndef AUTODRIVER_LIDAR_POSE_BUFFER_HPP_
#define AUTODRIVER_LIDAR_POSE_BUFFER_HPP_

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

#include <Eigen/Geometry>

#include "autodriver/lidar/motion_compensator.hpp"

namespace autodriver {
namespace lidar {

/**
 * @class autodriver::lidar::PoseBuffer
 * @brief Stores recent world←lidar poses; Lookup interpolates by time.
 */
class PoseBuffer {
public:
    /**
     * @brief Construct with a maximum number of stored poses.
     * @param capacity Older poses are dropped when exceeded.
     */
    explicit PoseBuffer(std::size_t capacity = 200);

    /** Append a pose stamped at @p time_ns (nanoseconds). */
    void Push(std::uint64_t time_ns, const Eigen::Affine3d& pose);

    /** Remove all stored poses. */
    void Clear();

    /** Bind Lookup to this buffer for MotionCompensator::SetPoseLookup. */
    PoseLookup AsLookup() const;

private:
    bool LookupUnlocked(std::uint64_t time_ns, Eigen::Affine3d* pose) const;

    mutable std::mutex mutex_;
    // Maximum number of (time, pose) samples.
    std::size_t capacity_;
    // Time-ordered pose history.
    std::deque<std::pair<std::uint64_t, Eigen::Affine3d>> poses_;
};

}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_POSE_BUFFER_HPP_

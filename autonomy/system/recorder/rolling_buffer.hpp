/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

/**
 * @file rolling_buffer.hpp
 * @brief L1 rolling window: bypass samples, channel-role flags, time prune.
 */

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy/system/recorder/proto/evidence.pb.h"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief Channel-role bits. Set on Attach; hot-path parse uses flags, not names.
 */
enum SampleFlag : uint16_t {
    kSampleOdom = 1u << 0,      ///< Odometry; parse geometry_msgs/Pose2D.
    kSamplePlan = 1u << 1,      ///< Global / local path.
    kSampleCmdVel = 1u << 2,    ///< /cmd_vel Twist or TwistStamped.
    kSampleBt = 1u << 3,        ///< Behavior-tree log.
    kSampleGoal = 1u << 4,      ///< Task / navigation goal.
    kSampleFeedback = 1u << 5,  ///< Task feedback (failure, loc quality).
    kSampleTeleop = 1u << 6,    ///< Teleop takeover.
    kSampleCost = 1u << 7,      ///< Explicit cost channel (not costmap name).
};

/**
 * @brief One bypass-captured message. payload is pre-serialized; never re-encode.
 */
struct Sample {
    std::string channel;       ///< Autolink topic name.
    std::string message_type;  ///< Fully-qualified protobuf type.
    std::string proto_desc;    ///< FileDescriptorSet; first sample of a channel only.
    std::string payload;       ///< Pre-serialized bytes; copied once from RawMessage.
    proto::DualClock clock;    ///< Monotonic + UTC.
    uint32_t sequence{0};      ///< Per-channel sequence.
    uint16_t flags{0};         ///< SampleFlag bitfield.
};

/**
 * @brief Fixed-duration ring. Push vs Snapshot/TakeAll are mutex-protected.
 *
 * Window length comes from RecorderOptions.window_sec. Flush should TakeAll
 * (moves payloads). Snapshot copies and is for read-only queries.
 */
class RollingBuffer {
public:
    /**
     * @brief Construct a buffer with a fixed window.
     * @param [in] window_ns: Retention window in nanoseconds.
     */
    explicit RollingBuffer(int64_t window_ns);

    /**
     * @brief Append a sample and prune entries older than the window.
     * @param [in] sample: Taken by value so the caller can move.
     */
    void Push(Sample sample);

    /**
     * @brief Copy every sample in the window. For queries; flush with TakeAll.
     * @return Copied samples.
     */
    std::vector<Sample> Snapshot() const;

    /**
     * @brief Move all buffered samples out; no payload copy.
     * @return Former contents; the buffer is empty afterwards.
     */
    std::vector<Sample> TakeAll();

    /**
     * @brief Drop samples at or before monotonic_ns.
     * @param [in] monotonic_ns: Cutoff monotonic time.
     */
    void DropThrough(int64_t monotonic_ns);

    /**
     * @brief Drop every sample.
     */
    void Clear();

    /**
     * @brief Number of samples currently buffered.
     * @return Sample count.
     */
    size_t size() const;

private:
    /**
     * @brief Drop expired samples; caller must hold mutex_.
     * @param [in] now_ns: Current monotonic time.
     */
    void PruneLocked(int64_t now_ns);

    int64_t window_ns_;           ///< Window length, nanoseconds.
    mutable std::mutex mutex_;    ///< Guards samples_.
    std::deque<Sample> samples_;  ///< Increasing time order.
};

}  // namespace recorder
}  // namespace system
}  // namespace autonomy

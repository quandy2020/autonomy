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
 * @file ring_buffer.hpp
 * @brief L1 rolling window: bypass samples, channel-role flags, time prune.
 */

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "common/proto/version_tuple.pb.h"

namespace autoloop {
namespace robot {

enum SampleFlag : uint16_t {
    kSampleOdom = 1u << 0,
    kSamplePlan = 1u << 1,
    kSampleCmdVel = 1u << 2,
    kSampleBt = 1u << 3,
    kSampleGoal = 1u << 4,
    kSampleFeedback = 1u << 5,
    kSampleTeleop = 1u << 6,
    kSampleCost = 1u << 7,
};

struct Sample {
    std::string channel;
    std::string message_type;
    std::string proto_desc;
    std::string payload;
    common::DualClock clock;
    uint32_t sequence{0};
    uint16_t flags{0};
};

class RingBuffer {
public:
    /**
     * @brief Construct a buffer with a fixed window.
     * @param [in] window_ns: Retention window in nanoseconds.
     */
    explicit RingBuffer(int64_t window_ns);

    /**
     * @brief Append a sample and prune entries older than the window.
     * @param [in] sample: Taken by value so the caller can move.
     */
    void Push(Sample sample);

    /**
     * @brief Copy current window (read-only query).
     */
    std::vector<Sample> Snapshot() const;

    /**
     * @brief Move out the window (flush path).
     */
    std::vector<Sample> TakeAll();

    size_t size() const;

private:
    void PruneLocked(int64_t now_ns);

    int64_t window_ns_{0};
    mutable std::mutex mu_;
    std::deque<Sample> q_;
};

}  // namespace robot
}  // namespace autoloop

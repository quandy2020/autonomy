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

/**
 * @file tracker.hpp
 * @brief Stable selected-person tracking over automsgs 2D detections.
 */

#ifndef AUTONOMY_PERCEPTION_SHADOW_TRACKER_HPP_
#define AUTONOMY_PERCEPTION_SHADOW_TRACKER_HPP_

#include "autonomy/perception/shadow/proto/shadow.pb.h"

#include <automsgs/msgs/vision_msgs/detection2d.pb.h>
#include <automsgs/msgs/vision_msgs/detection2d_array.pb.h>

#include <cstdint>
#include <memory>
#include <string>

namespace autonomy {
namespace perception {
namespace shadow {

/**
 * @brief Assigns stable IDs and maintains an explicitly selected person.
 *
 * Association uses high-confidence detections first, then unmatched
 * low-confidence detections to recover confirmed tracks. The implementation
 * keeps prediction state private and exposes only existing automsgs messages.
 */
class PersonTracker
{
public:
    explicit PersonTracker(const proto::ShadowOptions& options);
    ~PersonTracker();

    PersonTracker(const PersonTracker&) = delete;
    PersonTracker& operator=(const PersonTracker&) = delete;
    PersonTracker(PersonTracker&&) = delete;
    PersonTracker& operator=(PersonTracker&&) = delete;

    /**
     * @brief Associates one frame of person detections and writes stable IDs.
     * @param stamp_ns Monotonic frame timestamp in nanoseconds.
     * @param detections Task 2 detections, updated in place with track IDs.
     * @param error Optional diagnostic output, cleared on entry.
     * @return True when the frame is accepted.
     */
    bool Update(int64_t stamp_ns,
                automsgs::msgs::vision_msgs::Detection2DArray* detections,
                std::string* error = nullptr);

    /**
     * @brief Locks selection to exactly target_id, or clears it when empty.
     */
    void Select(const std::string& target_id);

    /**
     * @brief Returns the visible or short-term predicted selected detection.
     * @param detection Optional output; nullptr may be used as an availability
     * check.
     */
    bool Selected(automsgs::msgs::vision_msgs::Detection2D* detection) const;

    /**
     * @brief Returns confirmed tracks visible in the latest frame in ID order.
     */
    void Confirmed(
        automsgs::msgs::vision_msgs::Detection2DArray* detections) const;

    bool selected_visible() const;
    bool selected_predicted() const;

    /**
     * @brief Clears tracks and the selection without reusing issued IDs.
     */
    void Clear();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace shadow
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_SHADOW_TRACKER_HPP_

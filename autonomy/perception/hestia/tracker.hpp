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
 * @brief Multi-class IoU tracker for Hestia detections.
 */

#ifndef AUTONOMY_PERCEPTION_HESTIA_TRACKER_HPP_
#define AUTONOMY_PERCEPTION_HESTIA_TRACKER_HPP_

#include "autonomy/perception/hestia/proto/hestia.pb.h"

#include <automsgs/msgs/vision_msgs/detection2d_array.pb.h>

#include <cstdint>
#include <string>
#include <vector>

namespace autonomy {
namespace perception {
namespace hestia {

/**
 * @brief Assigns stable string track IDs across frames.
 */
class ObjectTracker
{
public:
    explicit ObjectTracker(const proto::HestiaOptions& options);

    /**
     * @brief Associates detections and writes Detection2D.id in place.
     * @param stamp_sec Frame timestamp in seconds.
     */
    void Update(double stamp_sec,
                automsgs::msgs::vision_msgs::Detection2DArray* detections);

    /** @brief Clears active tracks without resetting the ID counter. */
    void Clear();

private:
    struct Track {
        std::string id;
        std::string class_id;
        double center_x = 0.0;
        double center_y = 0.0;
        double width = 0.0;
        double height = 0.0;
        double last_stamp_sec = 0.0;
    };

    proto::HestiaOptions options_;
    std::vector<Track> tracks_;
    uint64_t next_id_ = 1;
};

}  // namespace hestia
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_HESTIA_TRACKER_HPP_

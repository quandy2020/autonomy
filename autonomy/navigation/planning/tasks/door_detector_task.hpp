/*
 * Copyright 2026 The Navigation Authors (navigation)
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
 * @file door_detector_task.hpp
 * @brief Geometric doorway detection from a depth column profile.
 */

#pragma once

#include "automsgs/msgs/sensor_msgs/camera_info.pb.h"
#include "automsgs/msgs/sensor_msgs/image.pb.h"

namespace navigation
{
namespace planning
{

/** @brief Doorway detection result. */
struct DoorDetection
{
    /** @brief True when a doorway was found. */
    bool detected{false};
    /** @brief Physical width in metres. */
    double width{0.0};
    /** @brief Door centre offset in the camera frame, metres. */
    double center_x{0.0};
    /** @brief Side-wall depth in metres. */
    double depth{0.0};
};

/** @brief Door detector thresholds. */
struct DoorDetectorOptions
{
    /** @brief Minimum accepted width in metres. */
    double min_width{0.4};
    /** @brief Maximum accepted width in metres. */
    double max_width{1.5};
    /** @brief Maximum considered depth in metres. */
    double max_depth{5.0};
    /** @brief Minimum side-wall depth in metres. */
    double min_side_depth{0.2};
};

class DoorDetectorTask
{
public:
    /** @brief Construct with width/depth thresholds. */
    explicit DoorDetectorTask(DoorDetectorOptions options = {});

    /**
     * @brief Detect a doorway in a depth image.
     *
     * @param[in] depth 32FC1 depth image.
     * @param[in] camera_info Intrinsics (k[0]=fx, k[2]=cx).
     * @param[out] detection Width/centre/depth on success.
     * @return false when no doorway matches the thresholds.
     */
    bool Detect(const automsgs::msgs::sensor_msgs::Image& depth,
                const automsgs::msgs::sensor_msgs::CameraInfo& camera_info,
                DoorDetection* detection) const;

private:
    /** @brief Detection thresholds. */
    DoorDetectorOptions options_;
};

}  // namespace planning
}  // namespace navigation

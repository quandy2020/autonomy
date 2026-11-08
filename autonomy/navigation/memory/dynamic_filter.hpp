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
 * @file dynamic_filter.hpp
 * @brief Removes moving points from a depth-derived point cloud.
 */

#pragma once

#include "automsgs/msgs/sensor_msgs/point_cloud.pb.h"

namespace navigation
{
namespace memory
{

/**
 * @brief Keeps points whose `dynamic_confidence` channel is below a threshold.
 *
 * @details The input is an automsgs `sensor_msgs::PointCloud`; the optional
 * channel `dynamic_confidence` (or `confidence`) carries a per-point score in
 * [0,1]. Points are kept when the channel is absent, and all channels are
 * copied 1:1 for the retained points.
 *
 * @par Threading
 * Stateless; safe to reuse across cycles on one thread.
 */
class DynamicFilter
{
public:
    /**
     * @brief Construct with the maximum kept dynamic score.
     *
     * @param[in] max_dynamic_confidence Threshold in [0,1].
     */
    explicit DynamicFilter(float max_dynamic_confidence = 0.6F);

    /**
     * @brief Keep static, finite points.
     *
     * @param[in] cloud Input cloud with optional confidence channel.
     * @return Filtered cloud with the same channel layout.
     */
    [[nodiscard]] automsgs::msgs::sensor_msgs::PointCloud Filter(
        const automsgs::msgs::sensor_msgs::PointCloud& cloud) const;

private:
    /** @brief Maximum dynamic score that is kept. */
    float max_dynamic_confidence_{0.6F};
};

}  // namespace memory
}  // namespace navigation

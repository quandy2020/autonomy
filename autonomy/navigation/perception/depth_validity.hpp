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
 * @file depth_validity.hpp
 * @brief Metric depth validity rules shared by preprocessing and tests.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "automsgs/msgs/sensor_msgs/image.pb.h"

namespace navigation
{
namespace perception
{

struct DepthStats
{
    /** @brief Total pixels inspected. */
    std::size_t pixel_count{0};
    /** @brief Pixels inside the valid depth range. */
    std::size_t valid_count{0};
    /** @brief valid_count / pixel_count in [0,1]. */
    double valid_ratio{0.0};
    /** @brief Minimum valid depth in metres. */
    double min_valid{0.0};
    /** @brief Maximum valid depth in metres. */
    double max_valid{0.0};
};

class DepthValidity
{
public:
    /**
     * @brief Construct with metric depth bounds.
     *
     * @param[in] min_depth Minimum valid depth in metres.
     * @param[in] max_depth Maximum valid depth in metres.
     */
    DepthValidity(double min_depth, double max_depth);

    /**
     * @brief Return true for finite depth inside [min,max].
     *
     * @param[in] depth Metric depth in metres.
     */
    [[nodiscard]] bool IsValid(double depth) const;

    /**
     * @brief Analyze raw depth pixels without resizing.
     *
     * @param[in] image 32FC1 or 16UC1 depth image.
     * @param[out] stats Pixel counts, ratio, min/max valid depth.
     * @return false for unsupported encoding or malformed payload.
     */
    bool Analyze(const automsgs::msgs::sensor_msgs::Image& image, DepthStats* stats) const;

private:
    /** @brief Minimum valid depth in metres. */
    double min_depth_{0.1};
    /** @brief Maximum valid depth in metres. */
    double max_depth_{8.0};
};

}  // namespace perception
}  // namespace navigation

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

#include "navigation/perception/depth_validity.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace navigation
{
namespace perception
{

DepthValidity::DepthValidity(double min_depth, double max_depth)
    : min_depth_(min_depth > 0.0 ? min_depth : 0.1),
      max_depth_(max_depth > min_depth_ ? max_depth : 8.0)
{
}

bool DepthValidity::IsValid(double depth) const
{
    return std::isfinite(depth) && depth >= min_depth_ && depth <= max_depth_;
}

bool DepthValidity::Analyze(const automsgs::msgs::sensor_msgs::Image& image,
                            DepthStats* stats) const
{
    if (stats == nullptr || image.width() == 0 || image.height() == 0 || image.data().empty())
    {
        return false;
    }
    const std::size_t count =
        static_cast<std::size_t>(image.width()) * static_cast<std::size_t>(image.height());
    stats->pixel_count = count;
    stats->valid_count = 0;
    stats->min_valid = std::numeric_limits<double>::infinity();
    stats->max_valid = 0.0;

    if (image.encoding() == "32FC1")
    {
        if (image.data().size() < count * sizeof(float))
        {
            return false;
        }
        const auto* values = reinterpret_cast<const float*>(image.data().data());
        for (std::size_t i = 0; i < count; ++i)
        {
            if (IsValid(values[i]))
            {
                ++stats->valid_count;
                stats->min_valid = std::min(stats->min_valid, static_cast<double>(values[i]));
                stats->max_valid = std::max(stats->max_valid, static_cast<double>(values[i]));
            }
        }
    }
    else if (image.encoding() == "16UC1" || image.encoding() == "mono16")
    {
        if (image.data().size() < count * sizeof(std::uint16_t))
        {
            return false;
        }
        const auto* values = reinterpret_cast<const std::uint16_t*>(image.data().data());
        for (std::size_t i = 0; i < count; ++i)
        {
            const double depth = static_cast<double>(values[i]) * 0.001;
            if (IsValid(depth))
            {
                ++stats->valid_count;
                stats->min_valid = std::min(stats->min_valid, depth);
                stats->max_valid = std::max(stats->max_valid, depth);
            }
        }
    }
    else
    {
        return false;
    }

    stats->valid_ratio = static_cast<double>(stats->valid_count) / static_cast<double>(count);
    if (stats->valid_count == 0)
    {
        stats->min_valid = 0.0;
        stats->max_valid = 0.0;
    }
    return true;
}

}  // namespace perception
}  // namespace navigation

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

#include "navigation/planning/tasks/door_detector_task.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace navigation
{
namespace planning
{

DoorDetectorTask::DoorDetectorTask(DoorDetectorOptions options) : options_(options)
{
}

bool DoorDetectorTask::Detect(const automsgs::msgs::sensor_msgs::Image& depth,
                              const automsgs::msgs::sensor_msgs::CameraInfo& camera_info,
                              DoorDetection* detection) const
{
    if (detection == nullptr || depth.width() == 0 || depth.height() == 0 ||
        depth.encoding() != "32FC1")
    {
        return false;
    }
    const std::size_t count =
        static_cast<std::size_t>(depth.width()) * depth.height();
    if (depth.data().size() < count * sizeof(float) || camera_info.k_size() < 3)
    {
        return false;
    }
    const auto* pixels = reinterpret_cast<const float*>(depth.data().data());
    const double fx = camera_info.k(0);
    if (!std::isfinite(fx) || fx <= 0.0)
    {
        return false;
    }

    std::vector<double> column_depth(depth.width(), 0.0);
    for (std::uint32_t x = 0; x < depth.width(); ++x)
    {
        double best = 0.0;
        for (std::uint32_t y = 0; y < depth.height(); ++y)
        {
            const double value =
                pixels[static_cast<std::size_t>(y) * depth.width() + x];
            if (std::isfinite(value) && value > 0.0 &&
                value <= options_.max_depth)
            {
                best = best == 0.0 ? value : std::min(best, value);
            }
        }
        column_depth[x] = best;
    }

    std::size_t best_start = 0;
    std::size_t best_length = 0;
    std::size_t run_start = 0;
    std::size_t run_length = 0;
    for (std::size_t x = 0; x <= column_depth.size(); ++x)
    {
        const bool invalid = x < column_depth.size() && column_depth[x] == 0.0;
        if (invalid)
        {
            if (run_length == 0)
            {
                run_start = x;
            }
            ++run_length;
        }
        else
        {
            if (run_length > best_length)
            {
                best_length = run_length;
                best_start = run_start;
            }
            run_length = 0;
        }
    }
    if (best_length == 0 || best_start == 0 ||
        best_start + best_length >= column_depth.size())
    {
        return false;
    }
    const double left_depth = column_depth[best_start - 1];
    const double right_depth = column_depth[best_start + best_length];
    const double side_depth = std::min(left_depth, right_depth);
    if (side_depth < options_.min_side_depth)
    {
        return false;
    }
    const double width = static_cast<double>(best_length) * side_depth / fx;
    if (width < options_.min_width || width > options_.max_width)
    {
        return false;
    }
    detection->detected = true;
    detection->width = width;
    detection->depth = side_depth;
    detection->center_x =
        (static_cast<double>(best_start) + 0.5 * best_length - camera_info.k(2)) *
        side_depth / fx;
    return true;
}

}  // namespace planning
}  // namespace navigation

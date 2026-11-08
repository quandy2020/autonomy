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

#include "navigation/memory/dynamic_filter.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace navigation
{
namespace memory
{
namespace
{

int FindConfidenceChannel(const automsgs::msgs::sensor_msgs::PointCloud& cloud)
{
    for (int i = 0; i < cloud.channels_size(); ++i)
    {
        const std::string& name = cloud.channels(i).name();
        if (name == "dynamic_confidence" || name == "confidence")
        {
            return i;
        }
    }
    return -1;
}

}  // namespace

DynamicFilter::DynamicFilter(float max_dynamic_confidence)
    : max_dynamic_confidence_(std::clamp(max_dynamic_confidence, 0.0F, 1.0F))
{
}

automsgs::msgs::sensor_msgs::PointCloud DynamicFilter::Filter(
    const automsgs::msgs::sensor_msgs::PointCloud& cloud) const
{
    automsgs::msgs::sensor_msgs::PointCloud filtered;
    *filtered.mutable_header() = cloud.header();
    for (int i = 0; i < cloud.channels_size(); ++i)
    {
        filtered.add_channels()->set_name(cloud.channels(i).name());
    }
    const int confidence_index = FindConfidenceChannel(cloud);
    for (int i = 0; i < cloud.points_size(); ++i)
    {
        const auto& point = cloud.points(i);
        if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
            !std::isfinite(point.z()))
        {
            continue;
        }
        float confidence = 0.0F;
        if (confidence_index >= 0 &&
            cloud.channels(confidence_index).values_size() == cloud.points_size())
        {
            confidence = cloud.channels(confidence_index).values(i);
        }
        if (confidence > max_dynamic_confidence_)
        {
            continue;
        }
        *filtered.add_points() = point;
        for (int channel = 0; channel < cloud.channels_size(); ++channel)
        {
            if (cloud.channels(channel).values_size() == cloud.points_size())
            {
                filtered.mutable_channels(channel)->add_values(
                    cloud.channels(channel).values(i));
            }
        }
    }
    return filtered;
}

}  // namespace memory
}  // namespace navigation

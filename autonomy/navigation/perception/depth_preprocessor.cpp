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

#include "navigation/perception/depth_preprocessor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace navigation
{
namespace perception
{
namespace
{

bool DecodeDepth(const automsgs::msgs::sensor_msgs::Image& image,
                 std::vector<float>* depth)
{
    const std::size_t count =
        static_cast<std::size_t>(image.width()) * static_cast<std::size_t>(image.height());
    if (count == 0)
    {
        return false;
    }
    depth->assign(count, 0.0F);
    if (image.encoding() == "32FC1")
    {
        if (image.data().size() < count * sizeof(float))
        {
            return false;
        }
        const auto* values = reinterpret_cast<const float*>(image.data().data());
        std::copy(values, values + count, depth->begin());
        return true;
    }
    if (image.encoding() == "16UC1" || image.encoding() == "mono16")
    {
        if (image.data().size() < count * sizeof(std::uint16_t))
        {
            return false;
        }
        const auto* values = reinterpret_cast<const std::uint16_t*>(image.data().data());
        for (std::size_t i = 0; i < count; ++i)
        {
            (*depth)[i] = static_cast<float>(values[i]) * 0.001F;
        }
        return true;
    }
    return false;
}

}  // namespace

DepthPreprocessor::DepthPreprocessor(DepthPreprocessorOptions options)
    : options_(options),
      validity_(options.min_depth, options.valid_max_depth)
{
}

bool DepthPreprocessor::Decode(const automsgs::msgs::sensor_msgs::Image& image,
                               std::vector<float>* metres)
{
    if (metres == nullptr)
    {
        return false;
    }
    return DecodeDepth(image, metres);
}

namespace
{

/** @brief One axis of a separable resize filter. */
struct AxisFilter
{
    std::vector<std::size_t> offsets{0};
    std::vector<int> indices;
    std::vector<double> weights;
};

/** @brief Build a triangular antialias filter when shrinking, bilinear when growing. */
AxisFilter BuildAxisFilter(std::uint32_t source, std::uint32_t target)
{
    AxisFilter filter;
    if (source == 0 || target == 0)
    {
        return filter;
    }
    const double scale = static_cast<double>(source) / target;
    const double radius = std::max(1.0, scale);
    for (std::uint32_t out = 0; out < target; ++out)
    {
        const double center = (static_cast<double>(out) + 0.5) * scale - 0.5;
        const int begin = static_cast<int>(std::floor(center - radius));
        const int end = static_cast<int>(std::ceil(center + radius));
        double sum = 0.0;
        std::vector<int> local_indices;
        std::vector<double> local_weights;
        for (int index = begin; index <= end; ++index)
        {
            const double weight =
                std::max(0.0, 1.0 - std::abs(static_cast<double>(index) - center) / radius);
            if (weight <= 0.0)
            {
                continue;
            }
            local_indices.push_back(std::clamp(index, 0, static_cast<int>(source) - 1));
            local_weights.push_back(weight);
            sum += weight;
        }
        if (sum <= 0.0)
        {
            local_indices.push_back(std::clamp(static_cast<int>(std::lround(center)), 0,
                                               static_cast<int>(source) - 1));
            local_weights.push_back(1.0);
            sum = 1.0;
        }
        for (std::size_t i = 0; i < local_weights.size(); ++i)
        {
            filter.indices.push_back(local_indices[i]);
            filter.weights.push_back(local_weights[i] / sum);
        }
        filter.offsets.push_back(filter.weights.size());
    }
    return filter;
}

/** @brief Separable resize of one single-channel image. */
void ResizeBilinear(const std::vector<float>& source, std::uint32_t source_w,
                    std::uint32_t source_h, std::uint32_t target_w,
                    std::uint32_t target_h, std::vector<float>* output)
{
    output->assign(static_cast<std::size_t>(target_w) * target_h, 0.0F);
    if (source.empty() || source_w == 0 || source_h == 0 || target_w == 0 || target_h == 0)
    {
        return;
    }
    const AxisFilter x_filter = BuildAxisFilter(source_w, target_w);
    const AxisFilter y_filter = BuildAxisFilter(source_h, target_h);
    for (std::uint32_t y = 0; y < target_h; ++y)
    {
        const auto y_begin = y_filter.offsets[y];
        const auto y_end = y_filter.offsets[y + 1];
        for (std::uint32_t x = 0; x < target_w; ++x)
        {
            const auto x_begin = x_filter.offsets[x];
            const auto x_end = x_filter.offsets[x + 1];
            double value = 0.0;
            for (std::size_t yi = y_begin; yi < y_end; ++yi)
            {
                const auto source_y = static_cast<std::size_t>(y_filter.indices[yi]);
                const double wy = y_filter.weights[yi];
                for (std::size_t xi = x_begin; xi < x_end; ++xi)
                {
                    const auto source_x = static_cast<std::size_t>(x_filter.indices[xi]);
                    value += wy * x_filter.weights[xi] *
                             source[source_y * source_w + source_x];
                }
            }
            (*output)[static_cast<std::size_t>(y) * target_w + x] =
                static_cast<float>(value);
        }
    }
}

}  // namespace

bool DepthPreprocessor::Process(const automsgs::msgs::sensor_msgs::Image& image,
                                std::vector<float>* chw, DepthStats* stats) const
{
    if (chw == nullptr || !validity_.Analyze(image, stats))
    {
        return false;
    }
    const std::uint32_t width = image.width();
    const std::uint32_t height = image.height();
    std::vector<float> depth;
    if (!DecodeDepth(image, &depth) || width == 0 || height == 0)
    {
        return false;
    }

    const std::size_t count = static_cast<std::size_t>(width) * height;
    double min_log = std::numeric_limits<double>::infinity();
    double max_log = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < count; ++i)
    {
        double value = depth[i];
        if (!std::isfinite(value))
        {
            value = 0.0;
        }
        value = std::clamp(value, 0.0, options_.max_depth_c1);
        depth[i] = static_cast<float>(value);
        const double log_depth = std::log1p(value);
        min_log = std::min(min_log, log_depth);
        max_log = std::max(max_log, log_depth);
    }
    if (!std::isfinite(min_log) || !std::isfinite(max_log))
    {
        return false;
    }

    const double c1_denominator = std::log1p(options_.max_depth_c1);
    const double c2_denominator = std::log1p(options_.max_depth_c2);
    std::array<std::vector<float>, 3> channels;
    for (auto& channel : channels)
    {
        channel.resize(count, 0.0F);
    }
    for (std::size_t i = 0; i < count; ++i)
    {
        const double log_depth = std::log1p(static_cast<double>(depth[i]));
        channels[0][i] = static_cast<float>(log_depth / c1_denominator);
        channels[1][i] = static_cast<float>(
            std::clamp(log_depth / c2_denominator, 0.0, 1.0));
        channels[2][i] = max_log > min_log
                             ? static_cast<float>((log_depth - min_log) /
                                                  (max_log - min_log))
                             : 0.0F;
    }

    const std::uint32_t size = options_.input_size;
    chw->assign(static_cast<std::size_t>(3) * size * size, 0.0F);
    for (std::size_t channel = 0; channel < channels.size(); ++channel)
    {
        std::vector<float> resized;
        ResizeBilinear(channels[channel], width, height, size, size, &resized);
        for (std::uint32_t i = 0; i < size * size; ++i)
        {
            const double value =
                (static_cast<double>(resized[i]) - options_.mean[channel]) /
                options_.std[channel];
            (*chw)[channel * size * size + i] = static_cast<float>(value);
        }
    }
    return true;
}

std::uint32_t DepthPreprocessor::InputSize() const
{
    return options_.input_size;
}

}  // namespace perception
}  // namespace navigation

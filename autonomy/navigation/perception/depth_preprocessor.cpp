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
#include <cmath>

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
      validity_(options.min_depth, options.max_depth)
{
}

bool DepthPreprocessor::Process(const automsgs::msgs::sensor_msgs::Image& image,
                                std::vector<float>* chw, DepthStats* stats) const
{
    if (chw == nullptr || !validity_.Analyze(image, stats))
    {
        return false;
    }
    std::vector<float> depth;
    if (!DecodeDepth(image, &depth))
    {
        return false;
    }

    const std::uint32_t size = options_.input_size;
    const std::uint32_t width = image.width();
    const std::uint32_t height = image.height();
    chw->assign(static_cast<std::size_t>(3) * size * size, 0.0F);

    const double scale = std::min(static_cast<double>(size) / width,
                                  static_cast<double>(size) / height);
    const std::uint32_t resized_w = std::max(1U, static_cast<std::uint32_t>(width * scale));
    const std::uint32_t resized_h = std::max(1U, static_cast<std::uint32_t>(height * scale));
    const std::uint32_t pad_x = (size - resized_w) / 2;
    const std::uint32_t pad_y = (size - resized_h) / 2;

    for (std::uint32_t y = 0; y < resized_h; ++y)
    {
        const double source_y = (static_cast<double>(y) + 0.5) / scale - 0.5;
        const int y0 = std::clamp(static_cast<int>(std::floor(source_y)), 0,
                                  static_cast<int>(height) - 1);
        const int y1 = std::min(y0 + 1, static_cast<int>(height) - 1);
        const double fy = source_y - std::floor(source_y);
        for (std::uint32_t x = 0; x < resized_w; ++x)
        {
            const double source_x = (static_cast<double>(x) + 0.5) / scale - 0.5;
            const int x0 = std::clamp(static_cast<int>(std::floor(source_x)), 0,
                                      static_cast<int>(width) - 1);
            const int x1 = std::min(x0 + 1, static_cast<int>(width) - 1);
            const double fx = source_x - std::floor(source_x);

            const auto sample = [&](int sx, int sy) -> double {
                const std::size_t index = static_cast<std::size_t>(sy) * width +
                                          static_cast<std::size_t>(sx);
                const double value = depth[index];
                return validity_.IsValid(value) ? value : 0.0;
            };
            const double top = sample(x0, y0) * (1.0 - fx) + sample(x1, y0) * fx;
            const double bottom = sample(x0, y1) * (1.0 - fx) + sample(x1, y1) * fx;
            const double depth = top * (1.0 - fy) + bottom * fy;
            if (depth <= 0.0)
            {
                continue;
            }
            const double clamped = std::clamp(depth, options_.min_depth, options_.max_depth);
            const double normalized =
                (clamped - options_.min_depth) / (options_.max_depth - options_.min_depth);
            const float value = static_cast<float>((normalized - options_.mean) / options_.std);
            const std::size_t output_index =
                static_cast<std::size_t>(y + pad_y) * size + (x + pad_x);
            for (std::size_t channel = 0; channel < 3; ++channel)
            {
                (*chw)[channel * size * size + output_index] = value;
            }
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

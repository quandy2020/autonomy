/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/perception/depth_statistics_encoder.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "automsgs/msgs/time_utils.hpp"

namespace navigation
{
namespace perception
{
namespace
{

constexpr std::uint32_t kTokenDim = 8;

float Pixel(const automsgs::msgs::sensor_msgs::Image& image, int x, int y)
{
    if (image.encoding() != "32FC1" || x < 0 || y < 0 ||
        x >= static_cast<int>(image.width()) ||
        y >= static_cast<int>(image.height()))
    {
        return 0.0F;
    }
    const std::size_t index =
        static_cast<std::size_t>(y) * image.width() + static_cast<std::size_t>(x);
    if (image.data().size() < (index + 1U) * sizeof(float))
    {
        return 0.0F;
    }
    return reinterpret_cast<const float*>(image.data().data())[index];
}

}  // namespace

common::Status DepthStatisticsEncoder::Encode(
    const observation::Observation& observation,
    observation::DepthFeature* feature)
{
    if (feature == nullptr)
    {
        return common::Status::InvalidArgument("feature must not be null");
    }
    const auto& depth = observation.depth();
    if (depth.width() == 0 || depth.height() == 0 || depth.data().empty())
    {
        return common::Status::InvalidArgument("depth image is empty");
    }
    const int cx = static_cast<int>(depth.width()) / 2;
    const int cy = static_cast<int>(depth.height()) / 2;
    const float centre = Pixel(depth, cx, cy);
    const float left = Pixel(depth, cx / 2, cy);
    const float right = Pixel(depth, cx + cx / 2, cy);
    const float top = Pixel(depth, cx, cy / 2);
    const float bottom = Pixel(depth, cx, cy + cy / 2);
    float minimum = 1e9F;
    float sum = 0.0F;
    std::uint32_t valid = 0;
    std::uint32_t total = 0;
    for (std::uint32_t y = 0; y < depth.height(); y += 4)
    {
        for (std::uint32_t x = 0; x < depth.width(); x += 4)
        {
            ++total;
            const float value = Pixel(depth, static_cast<int>(x), static_cast<int>(y));
            if (std::isfinite(value) && value > 0.05F && value < 20.0F)
            {
                minimum = std::min(minimum, value);
                sum += value;
                ++valid;
            }
        }
    }
    const float mean = valid > 0 ? sum / static_cast<float>(valid) : 0.0F;
    const float ratio = total > 0 ? static_cast<float>(valid) / static_cast<float>(total)
                                  : 0.0F;
    if (minimum > 1e8F)
    {
        minimum = 0.0F;
    }
    const float values[kTokenDim] = {centre, minimum, mean, ratio, left, right, top, bottom};
    feature->Clear();
    for (float value : values)
    {
        feature->add_tokens(value);
        feature->add_pooled(value);
    }
    feature->set_token_count(1);
    feature->set_token_dim(kTokenDim);
    feature->set_uncertainty(1.0F - ratio);
    feature->set_stamp_ns(static_cast<std::int64_t>(
        automsgs::msgs::builtin_interfaces::TimeToNanoseconds(
            observation.header().stamp())));
    feature->set_sequence(++sequence_);
    return common::Status{};
}

void DepthStatisticsEncoder::Reset()
{
    sequence_ = 0;
}

}  // namespace perception
}  // namespace navigation

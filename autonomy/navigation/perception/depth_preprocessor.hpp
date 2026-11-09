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
 * @file depth_preprocessor.hpp
 * @brief Depth decoding, DeFM metric-aware normalization and resize.
 */

#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "automsgs/msgs/sensor_msgs/image.pb.h"
#include "navigation/perception/depth_validity.hpp"

namespace navigation
{
namespace perception
{

/** @brief Depth preprocessing knobs. */
struct DepthPreprocessorOptions
{
    /** @brief Square model input side length in pixels. */
    std::uint32_t input_size{224};
    /** @brief Minimum valid depth in metres. */
    double min_depth{0.1};
    /** @brief Maximum depth counted as valid in statistics. */
    double valid_max_depth{100.0};
    /** @brief DeFM per-channel mean (C1, C2, C3). */
    std::array<double, 3> mean{0.248880, 0.495620, 0.492858};
    /** @brief DeFM per-channel standard deviation. */
    std::array<double, 3> std{0.139357, 0.271314, 0.297177};
    /** @brief C1 normalization scale. */
    double max_depth_c1{100.0};
    /** @brief C2 normalization scale (log1p(9) ~= log(10)). */
    double max_depth_c2{9.0};
};

class DepthPreprocessor
{
public:
    /** @brief Construct with preprocessing options. */
    explicit DepthPreprocessor(DepthPreprocessorOptions options = {});

    /**
     * @brief Convert an image to a normalized CHW float tensor.
     *
     * @param[in] image 32FC1 or 16UC1 depth image.
     * @param[out] chw 3 * input_size * input_size values, row-major.
     * @param[out] stats Validity statistics of the source image.
     * @return false for unsupported encoding or malformed payload.
     */
    bool Process(const automsgs::msgs::sensor_msgs::Image& image, std::vector<float>* chw,
                 DepthStats* stats) const;

    /**
     * @brief Decode a 32FC1 or 16UC1 depth image into metres.
     *
     * @param[in] image Depth image.
     * @param[out] metres width * height values, row-major.
     * @return false for unsupported encoding or malformed payload.
     */
    static bool Decode(const automsgs::msgs::sensor_msgs::Image& image,
                       std::vector<float>* metres);

    /** @brief Return the configured square input side length. */
    [[nodiscard]] std::uint32_t InputSize() const;

private:
    /** @brief Preprocessing options. */
    DepthPreprocessorOptions options_;
    /** @brief Validity checker using the same depth bounds. */
    DepthValidity validity_;
};

}  // namespace perception
}  // namespace navigation

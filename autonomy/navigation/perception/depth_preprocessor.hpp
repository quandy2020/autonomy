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
 * @brief Depth decoding, letterbox resize and normalization.
 */

#pragma once

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
    /** @brief Maximum valid depth in metres. */
    double max_depth{8.0};
    /** @brief Normalization mean. */
    double mean{0.5};
    /** @brief Normalization standard deviation (> 0). */
    double std{0.5};
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

/*
 * Copyright 2025 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/perception/fathom/processing/rgbd.hpp"

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace autonomy {
namespace perception {
namespace fathom {

namespace {

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

}  // namespace

bool PrepareRgbd(const DepthInput& input, int width, int height,
                 float depth_scale, common::network::TensorMap* tensors,
                 std::string* error) {
    if (tensors == nullptr) {
        SetError(error, "Fathom RGB-D tensor output is null.");
        return false;
    }
    if (input.bgr.empty() || input.raw_depth.empty()) {
        SetError(error, "Fathom RGB-D input images must not be empty.");
        return false;
    }
    if (input.bgr.type() != CV_8UC3 || input.raw_depth.type() != CV_16UC1) {
        SetError(error,
                 "Fathom RGB-D requires CV_8UC3 BGR and CV_16UC1 depth.");
        return false;
    }
    if (input.bgr.size() != input.raw_depth.size()) {
        SetError(error, "Fathom RGB and depth dimensions must match.");
        return false;
    }
    if (width <= 0 || height <= 0) {
        SetError(error, "Fathom RGB-D output dimensions must be positive.");
        return false;
    }
    if (!std::isfinite(depth_scale) || depth_scale <= 0.0F) {
        SetError(error, "Fathom depth scale must be finite and positive.");
        return false;
    }

    cv::Mat resized_bgr;
    cv::Mat resized_depth;
    cv::resize(input.bgr, resized_bgr, cv::Size(width, height), 0.0, 0.0,
               cv::INTER_LINEAR);
    cv::resize(input.raw_depth, resized_depth, cv::Size(width, height), 0.0,
               0.0, cv::INTER_NEAREST);

    cv::Mat rgb;
    cv::cvtColor(resized_bgr, rgb, cv::COLOR_BGR2RGB);
    const size_t plane_size = static_cast<size_t>(width) * height;
    std::vector<float> image(3 * plane_size);
    std::vector<float> depth(plane_size);
    for (int row = 0; row < height; ++row) {
        const cv::Vec3b* rgb_row = rgb.ptr<cv::Vec3b>(row);
        const uint16_t* depth_row = resized_depth.ptr<uint16_t>(row);
        for (int col = 0; col < width; ++col) {
            const size_t index = static_cast<size_t>(row) * width + col;
            const cv::Vec3b& pixel = rgb_row[col];
            image[index] = static_cast<float>(pixel[0]) / 255.0F;
            image[plane_size + index] = static_cast<float>(pixel[1]) / 255.0F;
            image[2 * plane_size + index] =
                static_cast<float>(pixel[2]) / 255.0F;
            depth[index] = static_cast<float>(depth_row[col]) * depth_scale;
        }
    }

    common::network::TensorMap prepared;
    prepared.emplace("image", common::network::Tensor::FromFloat32(
                                  std::move(image)));
    prepared.emplace("raw_depth", common::network::Tensor::FromFloat32(
                                      std::move(depth)));
    *tensors = std::move(prepared);
    return true;
}

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

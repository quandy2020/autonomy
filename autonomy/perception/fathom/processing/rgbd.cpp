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
#include <cstring>
#include <limits>
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

bool ValidateImage(const automsgs::msgs::sensor_msgs::Image& image,
                   const char* encoding, uint32_t bytes_per_pixel,
                   std::string* error) {
    if (image.width() == 0 || image.height() == 0) {
        SetError(error, "Fathom RGB-D image dimensions must be positive.");
        return false;
    }
    if (image.encoding() != encoding) {
        SetError(error, "Fathom RGB-D image encoding is unsupported.");
        return false;
    }
    if (image.is_bigendian()) {
        SetError(error, "Fathom RGB-D does not support big-endian images.");
        return false;
    }
    const size_t minimum_step =
        static_cast<size_t>(image.width()) * bytes_per_pixel;
    if (image.step() < minimum_step) {
        SetError(error, "Fathom RGB-D image step is too small.");
        return false;
    }
    const size_t required_data =
        static_cast<size_t>(image.height()) * image.step();
    if (image.data().size() < required_data) {
        SetError(error, "Fathom RGB-D image data is too small.");
        return false;
    }
    return true;
}

}  // namespace

bool PrepareRgbd(const automsgs::msgs::sensor_msgs::Image& rgb,
                 const automsgs::msgs::sensor_msgs::Image& raw_depth,
                 int width, int height,
                 float depth_scale, common::network::TensorMap* tensors,
                 std::string* error) {
    if (tensors == nullptr) {
        SetError(error, "Fathom RGB-D tensor output is null.");
        return false;
    }
    if (!ValidateImage(rgb, "bgr8", 3, error) ||
        !ValidateImage(raw_depth, "16UC1", 2, error)) {
        return false;
    }
    if (rgb.width() > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
        rgb.height() > static_cast<uint32_t>(std::numeric_limits<int>::max())) {
        SetError(error, "Fathom RGB-D image dimensions exceed OpenCV limits.");
        return false;
    }
    if (rgb.width() != raw_depth.width() || rgb.height() != raw_depth.height()) {
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

    cv::Mat bgr(static_cast<int>(rgb.height()), static_cast<int>(rgb.width()),
                CV_8UC3);
    cv::Mat raw_depth_mat(static_cast<int>(raw_depth.height()),
                          static_cast<int>(raw_depth.width()), CV_16UC1);
    for (uint32_t row = 0; row < rgb.height(); ++row) {
        std::memcpy(bgr.ptr(static_cast<int>(row)),
                    rgb.data().data() + static_cast<size_t>(row) * rgb.step(),
                    static_cast<size_t>(rgb.width()) * 3);
        std::memcpy(raw_depth_mat.ptr(static_cast<int>(row)),
                    raw_depth.data().data() +
                        static_cast<size_t>(row) * raw_depth.step(),
                    static_cast<size_t>(raw_depth.width()) * sizeof(uint16_t));
    }
    cv::Mat resized_bgr;
    cv::Mat resized_depth;
    cv::resize(bgr, resized_bgr, cv::Size(width, height), 0.0, 0.0,
               cv::INTER_LINEAR);
    cv::resize(raw_depth_mat, resized_depth, cv::Size(width, height), 0.0,
               0.0, cv::INTER_NEAREST);
    cv::Mat resized_rgb;
    cv::cvtColor(resized_bgr, resized_rgb, cv::COLOR_BGR2RGB);

    const size_t plane_size = static_cast<size_t>(width) * height;
    std::vector<float> image(3 * plane_size);
    std::vector<float> depth(plane_size);
    for (int row = 0; row < height; ++row) {
        for (int col = 0; col < width; ++col) {
            const size_t index = static_cast<size_t>(row) * width + col;
            const cv::Vec3b& pixel = resized_rgb.at<cv::Vec3b>(row, col);
            image[index] = static_cast<float>(pixel[0]) / 255.0F;
            image[plane_size + index] = static_cast<float>(pixel[1]) / 255.0F;
            image[2 * plane_size + index] =
                static_cast<float>(pixel[2]) / 255.0F;
            depth[index] =
                static_cast<float>(resized_depth.at<uint16_t>(row, col)) *
                depth_scale;
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

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

/**
 * @file rgbd.cpp
 * @brief Validation and tensor conversion for aligned RGB-D input frames.
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
                   uint32_t bytes_per_pixel, std::string* error) {
    if (image.width() == 0 || image.height() == 0) {
        SetError(error, "Fathom RGB-D image dimensions must be positive.");
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

bool ValidateRgbImage(const automsgs::msgs::sensor_msgs::Image& image,
                      std::string* error) {
    if (image.encoding() != "bgr8" && image.encoding() != "rgb8") {
        SetError(error, "Fathom RGB-D image encoding is unsupported.");
        return false;
    }
    return ValidateImage(image, 3, error);
}

bool ValidateDepthImage(const automsgs::msgs::sensor_msgs::Image& image,
                        std::string* error) {
    if (image.encoding() == "16UC1") {
        return ValidateImage(image, sizeof(uint16_t), error);
    }
    if (image.encoding() == "32FC1") {
        return ValidateImage(image, sizeof(float), error);
    }
    SetError(error, "Fathom RGB-D image encoding is unsupported.");
    return false;
}

float SanitizeDepth(float value) {
    return std::isfinite(value) && value > 0.0F ? value : 0.0F;
}

}  // namespace

bool PrepareRgbd(const automsgs::msgs::sensor_msgs::Image& rgb,
                 const automsgs::msgs::sensor_msgs::Image& raw_depth, int width,
                 int height, float depth_scale,
                 common::network::TensorMap* tensors, std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (tensors != nullptr) {
        tensors->clear();
    }
    if (tensors == nullptr) {
        SetError(error, "Fathom RGB-D tensor output is null.");
        return false;
    }
    if (!ValidateRgbImage(rgb, error) ||
        !ValidateDepthImage(raw_depth, error)) {
        return false;
    }
    if (rgb.width() > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
        rgb.height() > static_cast<uint32_t>(std::numeric_limits<int>::max())) {
        SetError(error, "Fathom RGB-D image dimensions exceed OpenCV limits.");
        return false;
    }
    if (rgb.width() != raw_depth.width() ||
        rgb.height() != raw_depth.height()) {
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

    cv::Mat source_rgb(static_cast<int>(rgb.height()),
                       static_cast<int>(rgb.width()), CV_8UC3);
    for (uint32_t row = 0; row < rgb.height(); ++row) {
        std::memcpy(source_rgb.ptr(static_cast<int>(row)),
                    rgb.data().data() + static_cast<size_t>(row) * rgb.step(),
                    static_cast<size_t>(rgb.width()) * 3);
    }
    cv::Mat source_depth;
    if (raw_depth.encoding() == "16UC1") {
        source_depth = cv::Mat(static_cast<int>(raw_depth.height()),
                               static_cast<int>(raw_depth.width()), CV_16UC1);
        for (uint32_t row = 0; row < raw_depth.height(); ++row) {
            std::memcpy(
                source_depth.ptr(static_cast<int>(row)),
                raw_depth.data().data() +
                    static_cast<size_t>(row) * raw_depth.step(),
                static_cast<size_t>(raw_depth.width()) * sizeof(uint16_t));
        }
    } else {
        source_depth = cv::Mat(static_cast<int>(raw_depth.height()),
                               static_cast<int>(raw_depth.width()), CV_32FC1);
        for (uint32_t row = 0; row < raw_depth.height(); ++row) {
            std::memcpy(source_depth.ptr(static_cast<int>(row)),
                        raw_depth.data().data() +
                            static_cast<size_t>(row) * raw_depth.step(),
                        static_cast<size_t>(raw_depth.width()) * sizeof(float));
            float* values = source_depth.ptr<float>(static_cast<int>(row));
            for (uint32_t col = 0; col < raw_depth.width(); ++col) {
                values[col] = SanitizeDepth(values[col]);
            }
        }
    }

    cv::Mat resized_source_rgb;
    cv::Mat resized_depth;
    cv::resize(source_rgb, resized_source_rgb, cv::Size(width, height), 0.0,
               0.0, cv::INTER_LINEAR);
    cv::resize(source_depth, resized_depth, cv::Size(width, height), 0.0, 0.0,
               cv::INTER_NEAREST);
    cv::Mat resized_rgb;
    if (rgb.encoding() == "bgr8") {
        cv::cvtColor(resized_source_rgb, resized_rgb, cv::COLOR_BGR2RGB);
    } else {
        resized_rgb = resized_source_rgb;
    }

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
            const float raw_value =
                raw_depth.encoding() == "16UC1"
                    ? static_cast<float>(resized_depth.at<uint16_t>(row, col))
                    : resized_depth.at<float>(row, col);
            depth[index] =
                SanitizeDepth(SanitizeDepth(raw_value) * depth_scale);
        }
    }

    common::network::TensorMap prepared;
    prepared.emplace("image",
                     common::network::Tensor::FromFloat32(std::move(image)));
    prepared.emplace("raw_depth",
                     common::network::Tensor::FromFloat32(std::move(depth)));
    *tensors = std::move(prepared);
    return true;
}

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

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
 * @file refiner.cpp
 * @brief End-to-end RGB-D preprocessing, inference, and reconstruction.
 */

#include "autonomy/perception/fathom/depth/refiner.hpp"

#include "autonomy/perception/fathom/processing/rgbd.hpp"
#include "autonomy/perception/fathom/projection/point_cloud.hpp"

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <utility>

namespace autonomy {
namespace perception {
namespace fathom {
namespace {

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error =
            message.rfind("Fathom: ", 0) == 0 ? message : "Fathom: " + message;
    }
}

bool OutputFloat32(const common::network::TensorMap& outputs, const char* name,
                   size_t expected_count, const float** values,
                   std::string* error) {
    const auto it = outputs.find(name);
    if (it == outputs.end()) {
        SetError(error, std::string("model output '") + name + "' is missing.");
        return false;
    }
    if (it->second.element_type() != common::network::ElementType::kFloat32) {
        SetError(error, std::string("model output '") + name +
                            "' must use float32 elements.");
        return false;
    }
    size_t count = 0;
    std::string tensor_error;
    if (!it->second.TryViewFloat32(values, &count, &tensor_error)) {
        SetError(error, tensor_error);
        return false;
    }
    if (count != expected_count) {
        SetError(error, std::string("model output '") + name +
                            "' has an unexpected element count.");
        return false;
    }
    return true;
}

automsgs::msgs::sensor_msgs::Image MakeFloatImage(
    const cv::Mat& depth, const automsgs::msgs::sensor_msgs::Image& reference) {
    automsgs::msgs::sensor_msgs::Image image;
    *image.mutable_header() = reference.header();
    image.set_height(static_cast<uint32_t>(depth.rows));
    image.set_width(static_cast<uint32_t>(depth.cols));
    image.set_encoding("32FC1");
    image.set_is_bigendian(false);
    image.set_step(static_cast<uint32_t>(depth.cols * sizeof(float)));
    const size_t bytes = static_cast<size_t>(image.height()) * image.step();
    image.mutable_data()->resize(bytes);
    std::memcpy(image.mutable_data()->data(), depth.ptr<float>(), bytes);
    return image;
}

automsgs::msgs::sensor_msgs::Image MakeValidityImage(
    const cv::Mat& validity,
    const automsgs::msgs::sensor_msgs::Image& reference, float threshold) {
    automsgs::msgs::sensor_msgs::Image image;
    *image.mutable_header() = reference.header();
    image.set_height(static_cast<uint32_t>(validity.rows));
    image.set_width(static_cast<uint32_t>(validity.cols));
    image.set_encoding("mono8");
    image.set_is_bigendian(false);
    image.set_step(static_cast<uint32_t>(validity.cols));
    image.mutable_data()->resize(static_cast<size_t>(image.height()) *
                                 image.step());
    for (int row = 0; row < validity.rows; ++row) {
        const float* source = validity.ptr<float>(row);
        char* destination = image.mutable_data()->data() +
                            static_cast<size_t>(row) * image.step();
        for (int col = 0; col < validity.cols; ++col) {
            destination[col] = static_cast<char>(
                std::isfinite(source[col]) && source[col] >= threshold ? 255
                                                                       : 0);
        }
    }
    return image;
}

}  // namespace

DepthRefiner::DepthRefiner(proto::FathomOptions options,
                           std::unique_ptr<FathomModelRunner> runner)
    : options_(std::move(options)), runner_(std::move(runner)) {}

std::unique_ptr<DepthRefiner> DepthRefiner::Create(
    const proto::FathomOptions& options,
    std::unique_ptr<FathomModelRunner> runner, std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (!ValidateModelOptions(options, error)) {
        return nullptr;
    }
    if (runner == nullptr) {
        SetError(error, "model runner is null.");
        return nullptr;
    }
    return std::unique_ptr<DepthRefiner>(
        new DepthRefiner(options, std::move(runner)));
}

bool DepthRefiner::Refine(
    const automsgs::msgs::sensor_msgs::Image& rgb,
    const automsgs::msgs::sensor_msgs::Image& raw_depth,
    const automsgs::msgs::sensor_msgs::CameraInfo& camera_info,
    automsgs::msgs::sensor_msgs::Image* refined_depth,
    automsgs::msgs::sensor_msgs::PointCloud2* point_cloud, std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (refined_depth != nullptr) {
        refined_depth->Clear();
    }
    if (point_cloud != nullptr) {
        point_cloud->Clear();
    }
    if (refined_depth == nullptr || point_cloud == nullptr) {
        SetError(error,
                 "refined_depth and point_cloud outputs must not be null.");
        return false;
    }

    common::network::TensorMap inputs;
    std::string detail;
    if (!PrepareRgbd(rgb, raw_depth, static_cast<int>(options_.input_width()),
                     static_cast<int>(options_.input_height()),
                     options_.depth_scale(), &inputs, &detail)) {
        SetError(error, detail);
        return false;
    }

    common::network::TensorMap outputs;
    if (!runner_->Run(inputs, &outputs, &detail)) {
        SetError(error, detail);
        return false;
    }

    const size_t profile_pixels = static_cast<size_t>(options_.input_width()) *
                                  static_cast<size_t>(options_.input_height());
    const float* profile_depth = nullptr;
    const float* profile_validity = nullptr;
    if (!OutputFloat32(outputs, "refined_depth", profile_pixels, &profile_depth,
                       error) ||
        !OutputFloat32(outputs, "validity", profile_pixels, &profile_validity,
                       error)) {
        return false;
    }
    if (rgb.width() > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
        rgb.height() > static_cast<uint32_t>(std::numeric_limits<int>::max())) {
        SetError(error, "input image dimensions exceed OpenCV limits.");
        return false;
    }

    cv::Mat profile_depth_mat(static_cast<int>(options_.input_height()),
                              static_cast<int>(options_.input_width()),
                              CV_32FC1, const_cast<float*>(profile_depth));
    cv::Mat profile_validity_mat(static_cast<int>(options_.input_height()),
                                 static_cast<int>(options_.input_width()),
                                 CV_32FC1,
                                 const_cast<float*>(profile_validity));
    cv::Mat restored_depth;
    cv::Mat restored_validity;
    cv::resize(
        profile_depth_mat, restored_depth,
        cv::Size(static_cast<int>(rgb.width()), static_cast<int>(rgb.height())),
        0.0, 0.0, cv::INTER_LINEAR);
    cv::resize(
        profile_validity_mat, restored_validity,
        cv::Size(static_cast<int>(rgb.width()), static_cast<int>(rgb.height())),
        0.0, 0.0, cv::INTER_LINEAR);

    const auto depth_image = MakeFloatImage(restored_depth, raw_depth);
    const auto validity_image = MakeValidityImage(restored_validity, raw_depth,
                                                  options_.mask_threshold());
    automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    if (!ProjectDepth(depth_image, validity_image, camera_info, &cloud,
                      &detail)) {
        SetError(error, detail);
        return false;
    }
    refined_depth->CopyFrom(depth_image);
    point_cloud->CopyFrom(cloud);
    return true;
}

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

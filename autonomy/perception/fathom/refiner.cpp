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

#include "autonomy/perception/fathom/refiner.hpp"

#include "autonomy/perception/fathom/point_cloud.hpp"
#include "autonomy/perception/fathom/rgbd.hpp"
#include "autonomy/perception/fathom/sky.hpp"

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

bool ResolveDepthOutput(const common::network::TensorMap& outputs,
                        size_t expected_count, const float** depth,
                        std::string* error) {
  if (OutputFloat32(outputs, "refined_depth", expected_count, depth, error)) {
    return true;
  }
  if (error != nullptr) {
    error->clear();
  }
  return OutputFloat32(outputs, "pred_depth", expected_count, depth, error);
}

std::vector<float> SynthesizeValidity(const float* depth, size_t count) {
  std::vector<float> validity(count);
  for (size_t i = 0; i < count; ++i) {
    validity[i] =
        (std::isfinite(depth[i]) && depth[i] > 0.0F) ? 1.0F : 0.0F;
  }
  return validity;
}

cv::Mat BgrForSky(const automsgs::msgs::sensor_msgs::Image& rgb, int width,
                  int height) {
  cv::Mat source(static_cast<int>(rgb.height()), static_cast<int>(rgb.width()),
                 CV_8UC3);
  for (uint32_t row = 0; row < rgb.height(); ++row) {
    std::memcpy(source.ptr(static_cast<int>(row)),
                rgb.data().data() + static_cast<size_t>(row) * rgb.step(),
                static_cast<size_t>(rgb.width()) * 3);
  }
  cv::Mat resized;
  cv::resize(source, resized, cv::Size(width, height), 0.0, 0.0,
             cv::INTER_LINEAR);
  if (rgb.encoding() == "rgb8") {
    cv::cvtColor(resized, resized, cv::COLOR_RGB2BGR);
  }
  return resized;
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
                     options_.depth_scale(), options_.max_depth_m(), &inputs,
                     &detail)) {
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
    if (!ResolveDepthOutput(outputs, profile_pixels, &profile_depth, error)) {
        return false;
    }

    std::vector<float> synthesized_validity;
    const float* profile_validity = nullptr;
    if (!OutputFloat32(outputs, "validity", profile_pixels, &profile_validity,
                       &detail)) {
        synthesized_validity = SynthesizeValidity(profile_depth, profile_pixels);
        profile_validity = synthesized_validity.data();
    }

    if (rgb.width() > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
        rgb.height() > static_cast<uint32_t>(std::numeric_limits<int>::max())) {
        SetError(error, "input image dimensions exceed OpenCV limits.");
        return false;
    }

    cv::Mat profile_depth_mat(static_cast<int>(options_.input_height()),
                              static_cast<int>(options_.input_width()),
                              CV_32FC1, const_cast<float*>(profile_depth));
    // Own a contiguous copy so sky correction can mutate safely.
    cv::Mat profile_depth_owned = profile_depth_mat.clone();
    if (options_.sky_correct().enabled()) {
        const cv::Mat bgr = BgrForSky(rgb, static_cast<int>(options_.input_width()),
                                      static_cast<int>(options_.input_height()));
        CorrectSkyFar(profile_depth_owned, bgr, options_.sky_correct(), nullptr);
    }

    cv::Mat profile_validity_mat(static_cast<int>(options_.input_height()),
                                 static_cast<int>(options_.input_width()),
                                 CV_32FC1,
                                 const_cast<float*>(profile_validity));
    cv::Mat restored_depth;
    cv::Mat restored_validity;
    cv::resize(
        profile_depth_owned, restored_depth,
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

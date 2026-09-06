/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file detector.hpp
 * @brief Fixed-prompt detector parameterized by open-vocab vs closed-set.
 */

#ifndef AUTONOMY_PERCEPTION_HESTIA_DETECTOR_HPP_
#define AUTONOMY_PERCEPTION_HESTIA_DETECTOR_HPP_

#include "autonomy/perception/hestia/proto/hestia.pb.h"
#include "autonomy/perception/hestia/runner.hpp"

#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/vision_msgs/detection2d_array.pb.h>
#include <google/protobuf/repeated_field.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autonomy {
namespace perception {
namespace hestia {

enum class PromptKind { OpenVocabulary, ClosedSet };

void SetPipelineError(std::string* error, const std::string& message);

std::vector<std::string> CopyClassNames(
    const google::protobuf::RepeatedPtrField<std::string>& class_names);

std::unique_ptr<Runner> CreateEngineRunner(const std::string& backend,
                                           const std::string& model_path,
                                           uint32_t width, uint32_t height,
                                           uint32_t max_detections,
                                           std::string* error);

bool InferDetections(
    const proto::HestiaOptions& options, Runner* runner,
    const std::vector<std::string>& class_names, uint32_t width,
    uint32_t height, const automsgs::msgs::sensor_msgs::Image& image,
    automsgs::msgs::vision_msgs::Detection2DArray* detections,
    std::string* error);

template <PromptKind K>
class Detector
{
public:
    static std::unique_ptr<Detector> Create(const proto::HestiaOptions& options,
                                            std::string* error = nullptr);

    static std::unique_ptr<Detector> Create(
        const proto::HestiaOptions& options, std::unique_ptr<Runner> runner,
        std::string* error = nullptr);

    Detector(const Detector&) = delete;
    Detector& operator=(const Detector&) = delete;

    bool Detect(const automsgs::msgs::sensor_msgs::Image& image,
                automsgs::msgs::vision_msgs::Detection2DArray* detections,
                std::string* error = nullptr) {
        return InferDetections(options_, runner_.get(), class_names_, width_,
                               height_, image, detections, error);
    }

private:
    Detector(proto::HestiaOptions options, std::unique_ptr<Runner> runner,
             std::vector<std::string> class_names, uint32_t width,
             uint32_t height)
        : options_(std::move(options)),
          runner_(std::move(runner)),
          class_names_(std::move(class_names)),
          width_(width),
          height_(height) {}

    static proto::HestiaOptions Normalize(proto::HestiaOptions options) {
        if constexpr (K == PromptKind::ClosedSet) {
            if (options.mode() != proto::MODE_DUAL) {
                options.set_mode(proto::MODE_DUAL);
            }
        }
        return options;
    }

    static const std::string& ModelPath(const proto::HestiaOptions& options) {
        if constexpr (K == PromptKind::OpenVocabulary) {
            return options.open_model_path();
        } else {
            return options.home_model_path();
        }
    }

    static uint32_t Width(const proto::HestiaOptions& options) {
        if constexpr (K == PromptKind::OpenVocabulary) {
            return options.open_width();
        } else {
            return options.home_width();
        }
    }

    static uint32_t Height(const proto::HestiaOptions& options) {
        if constexpr (K == PromptKind::OpenVocabulary) {
            return options.open_height();
        } else {
            return options.home_height();
        }
    }

    static const google::protobuf::RepeatedPtrField<std::string>& ClassNames(
        const proto::HestiaOptions& options) {
        if constexpr (K == PromptKind::OpenVocabulary) {
            return options.open_prompts();
        } else {
            return options.home_labels();
        }
    }

    proto::HestiaOptions options_;
    std::unique_ptr<Runner> runner_;
    std::vector<std::string> class_names_;
    uint32_t width_ = 0;
    uint32_t height_ = 0;
};

using OpenDetector = Detector<PromptKind::OpenVocabulary>;
using ClosedDetector = Detector<PromptKind::ClosedSet>;

}  // namespace hestia
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_HESTIA_DETECTOR_HPP_

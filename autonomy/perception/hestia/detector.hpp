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
 * @brief Open and home YOLO-style detectors with injectable runners.
 */

#ifndef AUTONOMY_PERCEPTION_HESTIA_DETECTOR_HPP_
#define AUTONOMY_PERCEPTION_HESTIA_DETECTOR_HPP_

#include "autonomy/common/network/common/tensor.hpp"
#include "autonomy/perception/hestia/proto/hestia.pb.h"

#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/vision_msgs/detection2d_array.pb.h>

#include <memory>
#include <string>
#include <vector>

namespace autonomy {
namespace perception {
namespace hestia {

/**
 * @brief Backend-independent inference contract used by Hestia detectors.
 */
class DetectorRunner
{
public:
    virtual ~DetectorRunner() = default;

    virtual bool Run(const common::network::TensorMap& inputs,
                     common::network::TensorMap* outputs,
                     std::string* error = nullptr) = 0;
};

/**
 * @brief Open-vocabulary fixed-profile detector.
 *
 * `SetPrompts` only updates the string table used to map class indices to
 * `ObjectHypothesis.class_id`. The ONNX/TensorRT artifact remains frozen to
 * the prompt list used at export; re-export when changing the vocabulary.
 */
class OpenDetector
{
public:
    static std::unique_ptr<OpenDetector> Create(
        const proto::HestiaOptions& options, std::string* error = nullptr);

    static std::unique_ptr<OpenDetector> Create(
        const proto::HestiaOptions& options,
        std::unique_ptr<DetectorRunner> runner, std::string* error = nullptr);

    OpenDetector(const OpenDetector&) = delete;
    OpenDetector& operator=(const OpenDetector&) = delete;

    void SetPrompts(const std::vector<std::string>& prompts);

    bool Detect(const automsgs::msgs::sensor_msgs::Image& image,
                automsgs::msgs::vision_msgs::Detection2DArray* detections,
                std::string* error = nullptr);

private:
    OpenDetector(proto::HestiaOptions options,
                 std::unique_ptr<DetectorRunner> runner,
                 std::vector<std::string> labels);

    proto::HestiaOptions options_;
    std::unique_ptr<DetectorRunner> runner_;
    std::vector<std::string> labels_;
};

/**
 * @brief Closed-set home ontology detector for dual-mode fast path.
 */
class HomeDetector
{
public:
    static std::unique_ptr<HomeDetector> Create(
        const proto::HestiaOptions& options, std::string* error = nullptr);

    static std::unique_ptr<HomeDetector> Create(
        const proto::HestiaOptions& options,
        std::unique_ptr<DetectorRunner> runner, std::string* error = nullptr);

    HomeDetector(const HomeDetector&) = delete;
    HomeDetector& operator=(const HomeDetector&) = delete;

    bool Detect(const automsgs::msgs::sensor_msgs::Image& image,
                automsgs::msgs::vision_msgs::Detection2DArray* detections,
                std::string* error = nullptr);

private:
    HomeDetector(proto::HestiaOptions options,
                 std::unique_ptr<DetectorRunner> runner,
                 std::vector<std::string> labels);

    proto::HestiaOptions options_;
    std::unique_ptr<DetectorRunner> runner_;
    std::vector<std::string> labels_;
};

}  // namespace hestia
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_HESTIA_DETECTOR_HPP_

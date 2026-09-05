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
 * @brief YOLO26 person detection with automsgs transport output.
 */

#ifndef AUTONOMY_PERCEPTION_SHADOW_DETECTOR_HPP_
#define AUTONOMY_PERCEPTION_SHADOW_DETECTOR_HPP_

#include "autonomy/common/network/common/tensor.hpp"
#include "autonomy/perception/shadow/proto/shadow.pb.h"

#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/vision_msgs/detection2d_array.pb.h>

#include <memory>
#include <string>

namespace autonomy {
namespace perception {
namespace shadow {

/**
 * @brief Backend-independent inference contract used by YoloDetector.
 */
class DetectorRunner
{
public:
    virtual ~DetectorRunner() = default;

    /**
     * @brief Executes one inference request.
     * @param inputs Named model input tensors.
     * @param outputs Named model output tensors, populated on success.
     * @param error Optional diagnostic output.
     * @return True when inference succeeds.
     */
    virtual bool Run(const common::network::TensorMap& inputs,
                     common::network::TensorMap* outputs,
                     std::string* error = nullptr) = 0;
};

/**
 * @brief Preprocesses RGB images and parses fixed-profile YOLO26 detections.
 *
 * The public boundary uses the existing automsgs Detection2DArray so stable
 * IDs can be assigned by PersonTracker without introducing another transport
 * message type.
 */
class YoloDetector
{
public:
    /**
     * @brief Creates the common-network-backed detector for a fixed profile.
     * @param options Validated Shadow runtime configuration.
     * @param error Optional diagnostic output, cleared on entry.
     * @return A ready detector, or nullptr when setup fails.
     */
    static std::unique_ptr<YoloDetector> Create(
        const proto::ShadowOptions& options, std::string* error = nullptr);

    /**
     * @brief Creates a detector with an injected runner for tests or adapters.
     * @param options Validated Shadow runtime configuration.
     * @param runner Concrete or test inference runner; ownership transfers.
     * @param error Optional diagnostic output, cleared on entry.
     * @return A ready detector, or nullptr when setup fails.
     */
    static std::unique_ptr<YoloDetector> Create(
        const proto::ShadowOptions& options,
        std::unique_ptr<DetectorRunner> runner, std::string* error = nullptr);

    YoloDetector(const YoloDetector&) = delete;
    YoloDetector& operator=(const YoloDetector&) = delete;
    YoloDetector(YoloDetector&&) = delete;
    YoloDetector& operator=(YoloDetector&&) = delete;

    /**
     * @brief Detects configured person-class boxes in one RGB image.
     *
     * The output is cleared before processing and assigned only after
     * preprocessing, inference, and output validation succeed. Detection IDs
     * remain empty until PersonTracker assigns stable identities.
     *
     * @param image Source `rgb8` or `bgr8` image.
     * @param detections Existing automsgs detection output.
     * @param error Optional diagnostic output, cleared on entry.
     * @return True when the frame was processed, including no-detection frames.
     */
    bool Detect(const automsgs::msgs::sensor_msgs::Image& image,
                automsgs::msgs::vision_msgs::Detection2DArray* detections,
                std::string* error = nullptr);

private:
    YoloDetector(proto::ShadowOptions options,
                 std::unique_ptr<DetectorRunner> runner);

    proto::ShadowOptions options_;
    std::unique_ptr<DetectorRunner> runner_;
};

}  // namespace shadow
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_SHADOW_DETECTOR_HPP_

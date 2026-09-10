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
 * @file base_component.hpp
 * @brief Autolink component for YOLO26 base perception tasks.
 */

#ifndef AUTONOMY_PERCEPTION_BASE_BASE_COMPONENT_HPP_
#define AUTONOMY_PERCEPTION_BASE_BASE_COMPONENT_HPP_

#include "autonomy/perception/base/engine/model.hpp"
#include "autonomy/perception/base/frame.hpp"
#include "autonomy/perception/base/proto/base.pb.h"

#include "autolink/component/component.hpp"

#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/vision_msgs/classification.pb.h>
#include <automsgs/msgs/vision_msgs/detection2d_array.pb.h>

#include <functional>
#include <memory>
#include <string>

namespace autonomy {
namespace perception {
namespace base {

class ComponentTestApi;

class Component final
    : public autolink::Component<automsgs::msgs::sensor_msgs::Image>
{
public:
    using Image = automsgs::msgs::sensor_msgs::Image;
    using Detection2DArray = automsgs::msgs::vision_msgs::Detection2DArray;
    using Classification = automsgs::msgs::vision_msgs::Classification;

    ~Component() override;

    bool Init() override;

    bool Proc(const std::shared_ptr<Image>& rgb) override;

protected:
    void Clear() override;

private:
    friend class ComponentTestApi;

    using FrameFunction =
        std::function<bool(const Image&, Outputs*, std::string*)>;
    using DetectionPublisher = std::function<bool(const Detection2DArray&)>;
    using ClassificationPublisher = std::function<bool(const Classification&)>;
    using DepthPublisher = std::function<bool(const Image&)>;

    bool ProcessFrame(const Image& rgb, Outputs* outputs, std::string* error);

    proto::BaseOptions options_;
    std::unique_ptr<Model> detect_;
    std::unique_ptr<Model> segment_;
    std::unique_ptr<Model> classify_;
    std::unique_ptr<Model> pose_;
    std::unique_ptr<Model> obb_;
    std::unique_ptr<Model> track_;
    std::unique_ptr<Model> depth_;
    std::shared_ptr<autolink::Writer<Detection2DArray>> detections_writer_;
    std::shared_ptr<autolink::Writer<Detection2DArray>> masks_writer_;
    std::shared_ptr<autolink::Writer<Classification>> classification_writer_;
    std::shared_ptr<autolink::Writer<Detection2DArray>> poses_writer_;
    std::shared_ptr<autolink::Writer<Detection2DArray>> obb_writer_;
    std::shared_ptr<autolink::Writer<Detection2DArray>> tracks_writer_;
    std::shared_ptr<autolink::Writer<Image>> depth_writer_;
    FrameFunction frame_;
    DetectionPublisher publish_detections_;
    DetectionPublisher publish_masks_;
    ClassificationPublisher publish_classification_;
    DetectionPublisher publish_poses_;
    DetectionPublisher publish_obb_;
    DetectionPublisher publish_tracks_;
    DepthPublisher publish_depth_;
};

}  // namespace base
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_BASE_BASE_COMPONENT_HPP_

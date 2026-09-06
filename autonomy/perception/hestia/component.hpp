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
 * @file hestia_component.hpp
 * @brief Autolink component for open-vocabulary home perception.
 */

#ifndef AUTONOMY_PERCEPTION_HESTIA_HESTIA_COMPONENT_HPP_
#define AUTONOMY_PERCEPTION_HESTIA_HESTIA_COMPONENT_HPP_

#include "autonomy/perception/hestia/detector.hpp"
#include "autonomy/perception/hestia/lifter.hpp"
#include "autonomy/perception/hestia/merger.hpp"
#include "autonomy/perception/hestia/open_worker.hpp"
#include "autonomy/perception/hestia/tracker.hpp"

#include "autolink/component/component.hpp"

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/vision_msgs/detection2d_array.pb.h>
#include <automsgs/msgs/vision_msgs/detection3d_array.pb.h>

#include <functional>
#include <memory>
#include <string>

namespace autonomy {
namespace perception {
namespace hestia {

class HestiaComponentTestApi;

/**
 * @brief Detects home objects from synchronized RGB-D and publishes 2D/3D boxes.
 */
class HestiaComponent final
    : public autolink::Component<automsgs::msgs::sensor_msgs::Image,
                                 automsgs::msgs::sensor_msgs::Image,
                                 automsgs::msgs::sensor_msgs::CameraInfo>
{
public:
    using Image = automsgs::msgs::sensor_msgs::Image;
    using CameraInfo = automsgs::msgs::sensor_msgs::CameraInfo;
    using Detection2DArray = automsgs::msgs::vision_msgs::Detection2DArray;
    using Detection3DArray = automsgs::msgs::vision_msgs::Detection3DArray;

    ~HestiaComponent() override;

    bool Init() override;

    bool Proc(const std::shared_ptr<Image>& rgb,
              const std::shared_ptr<Image>& depth,
              const std::shared_ptr<CameraInfo>& camera_info) override;

protected:
    void Clear() override;

private:
    friend class HestiaComponentTestApi;

    using FrameFunction = std::function<bool(
        const Image&, const Image&, const CameraInfo&, Detection2DArray*,
        Detection3DArray*, std::string*)>;
    using Detection2dPublisher = std::function<bool(const Detection2DArray&)>;
    using Detection3dPublisher = std::function<bool(const Detection3DArray&)>;

    bool ProcessFrame(const Image& rgb, const Image& depth,
                      const CameraInfo& camera_info,
                      Detection2DArray* detections_2d,
                      Detection3DArray* detections_3d,
                      std::string* error);

    proto::HestiaOptions options_;
    std::unique_ptr<OpenDetector> open_detector_;
    std::unique_ptr<HomeDetector> home_detector_;
    std::unique_ptr<DepthLifter> lifter_;
    std::unique_ptr<ObjectTracker> tracker_;
    std::unique_ptr<DetectionMerger> merger_;
    std::unique_ptr<OpenAsyncWorker> open_worker_;
    std::shared_ptr<autolink::Writer<Detection2DArray>> detections_2d_writer_;
    std::shared_ptr<autolink::Writer<Detection3DArray>> detections_3d_writer_;
    FrameFunction frame_;
    Detection2dPublisher publish_2d_;
    Detection3dPublisher publish_3d_;
};

}  // namespace hestia
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_HESTIA_HESTIA_COMPONENT_HPP_

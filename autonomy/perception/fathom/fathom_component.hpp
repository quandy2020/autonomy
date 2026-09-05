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

#ifndef AUTONOMY_PERCEPTION_FATHOM_FATHOM_COMPONENT_HPP_
#define AUTONOMY_PERCEPTION_FATHOM_FATHOM_COMPONENT_HPP_

#include "autonomy/perception/fathom/fathom_node_runner.hpp"

#include "autolink/component/component.hpp"

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include <functional>
#include <memory>
#include <string>

namespace autonomy {
namespace perception {
namespace fathom {

class FathomComponentTestApi;

/** Autolink entrypoint for synchronized RGB-D refinement and publication. */
class FathomComponent final
    : public autolink::Component<automsgs::msgs::sensor_msgs::Image,
                                 automsgs::msgs::sensor_msgs::Image,
                                 automsgs::msgs::sensor_msgs::CameraInfo> {
public:
    using Image = automsgs::msgs::sensor_msgs::Image;
    using CameraInfo = automsgs::msgs::sensor_msgs::CameraInfo;
    using PointCloud2 = automsgs::msgs::sensor_msgs::PointCloud2;

    ~FathomComponent() override = default;

    bool Init() override;
    bool Proc(const std::shared_ptr<Image>& rgb,
              const std::shared_ptr<Image>& raw_depth,
              const std::shared_ptr<CameraInfo>& camera_info) override;

protected:
    void Clear() override;

private:
    friend class FathomComponentTestApi;

    using RunnerFunction = std::function<bool(
        const Image&, const Image&, const CameraInfo&, Image*, PointCloud2*,
        std::string*)>;
    using ImagePublisher = std::function<bool(const Image&)>;
    using PointCloudPublisher = std::function<bool(const PointCloud2&)>;

    std::unique_ptr<FathomNodeRunner> runner_;
    std::shared_ptr<autolink::Writer<Image>> refined_depth_writer_;
    std::shared_ptr<autolink::Writer<PointCloud2>> point_cloud_writer_;
    RunnerFunction runner_process_;
    ImagePublisher refined_depth_publish_;
    PointCloudPublisher point_cloud_publish_;
};

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FATHOM_FATHOM_COMPONENT_HPP_

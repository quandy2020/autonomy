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

#ifndef AUTONOMY_PERCEPTION_FOLLOW_FOLLOW_COMPONENT_HPP_
#define AUTONOMY_PERCEPTION_FOLLOW_FOLLOW_COMPONENT_HPP_

#include "autonomy/perception/follow/grid.hpp"
#include "autonomy/perception/follow/localizer.hpp"
#include "autonomy/perception/follow/planner.hpp"
#include "autonomy/perception/follow/proto/follow.pb.h"
#include "autonomy/transform/buffer.hpp"

#include "autolink/component/component.hpp"

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/map_msgs/grid_map.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/std_msgs/string.pb.h>
#include <automsgs/msgs/vision_msgs/detection2d_array.pb.h>

#include <memory>
#include <mutex>
#include <string>

namespace autonomy {
namespace perception {
namespace follow {

class FollowComponent final
    : public autolink::Component<automsgs::msgs::vision_msgs::Detection2DArray,
                                 automsgs::msgs::sensor_msgs::Image,
                                 automsgs::msgs::sensor_msgs::CameraInfo,
                                 automsgs::msgs::nav_msgs::Odometry>
{
public:
    using Tracks = automsgs::msgs::vision_msgs::Detection2DArray;
    using Image = automsgs::msgs::sensor_msgs::Image;
    using CameraInfo = automsgs::msgs::sensor_msgs::CameraInfo;
    using Odometry = automsgs::msgs::nav_msgs::Odometry;

    ~FollowComponent() override;

    bool Init() override;

    bool Proc(const std::shared_ptr<Tracks>& tracks,
              const std::shared_ptr<Image>& depth,
              const std::shared_ptr<CameraInfo>& camera_info,
              const std::shared_ptr<Odometry>& odom) override;

protected:
    void Clear() override;

private:
    void OnSelect(const std::shared_ptr<automsgs::msgs::std_msgs::String>& msg);

    bool LookupCameraToMap(
        const Image& depth,
        automsgs::msgs::geometry_msgs::TransformStamped* transform) const;

    const automsgs::msgs::vision_msgs::Detection2D* SelectTrack(
        const Tracks& tracks) const;

    proto::FollowOptions options_;
    std::unique_ptr<Localizer> localizer_;
    std::unique_ptr<LocalGrid> grid_;
    std::unique_ptr<Planner> planner_;
    transform::Buffer* tf_buffer_ = nullptr;

    std::shared_ptr<autolink::Writer<automsgs::msgs::geometry_msgs::PoseStamped>>
        target_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Path>>
        path_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::map_msgs::GridMap>>
        grid_writer_;
    std::shared_ptr<autolink::Reader<automsgs::msgs::std_msgs::String>>
        select_reader_;

    mutable std::mutex select_mutex_;
    std::string selected_id_;
};

}  // namespace follow
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FOLLOW_FOLLOW_COMPONENT_HPP_

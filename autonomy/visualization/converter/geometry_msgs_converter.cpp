/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/visualization/converter/geometry_msgs_converter.hpp"

#include "autolink/autolink.hpp"
#include "autonomy/visualization/converter/converter_detail.hpp"

namespace autonomy {
namespace visualization {
namespace converter {
namespace impl {

using detail::CreateColor;
using detail::CreatePose;
using detail::ExtractFrameId;
using detail::ExtractTimestamp;
using detail::SetEntityHeader;

foxglove::schemas::SceneUpdate ToFoxgloveImpl(
    const autonomy::commsgs::proto::geometry_msgs::PoseStamped& message) {
    foxglove::schemas::SceneUpdate scene_update;

    // 创建场景实体
    foxglove::schemas::SceneEntity entity;
    SetEntityHeader(entity, ExtractTimestamp(message), ExtractFrameId(message));

    entity.id = "pose_stamped";
    entity.frame_locked = false;

    // 显示位姿（使用箭头表示方向）
    if (message.has_pose()) {
        const auto& pose = message.pose();

        // 创建箭头显示位姿方向
        foxglove::schemas::ArrowPrimitive arrow;
        arrow.pose = CreatePose(pose);

        // 设置箭头大小
        arrow.shaft_length = 0.5;
        arrow.head_length = 0.1;
        arrow.shaft_diameter = 0.05;
        arrow.head_diameter = 0.15;

        // 设置颜色（蓝色）
        arrow.color = CreateColor(0.0, 0.0, 1.0, 1.0);

        entity.arrows.push_back(arrow);
    }

    if (!entity.arrows.empty()) {
        scene_update.entities.push_back(entity);
    }

    AINFO << "Converted PoseStamped to SceneUpdate";

    return scene_update;
}

foxglove::schemas::SceneUpdate ToFoxgloveImpl(
    const autonomy::commsgs::proto::geometry_msgs::PoseArray& message) {
    foxglove::schemas::SceneUpdate scene_update;

    if (message.poses_size() == 0) {
        AWARN << "PoseArray is empty, returning empty SceneUpdate";
        return scene_update;
    }

    // 创建场景实体
    foxglove::schemas::SceneEntity entity;
    SetEntityHeader(entity, ExtractTimestamp(message), ExtractFrameId(message));

    entity.id = "pose_array";
    entity.frame_locked = false;

    // 为每个位姿创建箭头
    for (int i = 0; i < message.poses_size(); ++i) {
        const auto& pose = message.poses(i);

        foxglove::schemas::ArrowPrimitive arrow;
        arrow.pose = CreatePose(pose);

        arrow.shaft_length = 0.3;
        arrow.head_length = 0.08;
        arrow.shaft_diameter = 0.03;
        arrow.head_diameter = 0.1;

        // 使用不同颜色区分不同的位姿
        foxglove::schemas::Color color;
        double hue = static_cast<double>(i) / message.poses_size();
        color.r = (hue < 0.5) ? (1.0 - hue * 2.0) : 0.0;
        color.g = (hue < 0.5) ? (hue * 2.0) : (2.0 - hue * 2.0);
        color.b = (hue >= 0.5) ? ((hue - 0.5) * 2.0) : 0.0;
        color.a = 1.0;
        arrow.color = color;

        entity.arrows.push_back(arrow);
    }

    scene_update.entities.push_back(entity);

    AINFO << "Converted PoseArray with " << message.poses_size()
          << " poses to SceneUpdate";

    return scene_update;
}

foxglove::schemas::SceneUpdate ToFoxgloveImpl(
    const autonomy::commsgs::proto::geometry_msgs::TransformStamped& message) {
    foxglove::schemas::SceneUpdate scene_update;

    // 创建场景实体
    foxglove::schemas::SceneEntity entity;
    SetEntityHeader(entity, ExtractTimestamp(message), ExtractFrameId(message));

    entity.id = "transform_" + message.child_frame_id();
    entity.frame_locked = false;

    // 显示坐标变换（使用箭头）
    if (message.has_transform()) {
        const auto& transform = message.transform();

        // 创建 Pose（从 Transform 转换）
        autonomy::commsgs::proto::geometry_msgs::Pose pose;
        pose.mutable_position()->set_x(transform.translation().x());
        pose.mutable_position()->set_y(transform.translation().y());
        pose.mutable_position()->set_z(transform.translation().z());
        pose.mutable_orientation()->set_x(transform.rotation().x());
        pose.mutable_orientation()->set_y(transform.rotation().y());
        pose.mutable_orientation()->set_z(transform.rotation().z());
        pose.mutable_orientation()->set_w(transform.rotation().w());

        foxglove::schemas::ArrowPrimitive arrow;
        arrow.pose = CreatePose(pose);

        arrow.shaft_length = 0.4;
        arrow.head_length = 0.1;
        arrow.shaft_diameter = 0.04;
        arrow.head_diameter = 0.12;

        // 设置颜色（紫色）
        arrow.color = CreateColor(1.0, 0.0, 1.0, 1.0);

        entity.arrows.push_back(arrow);
    }

    if (!entity.arrows.empty()) {
        scene_update.entities.push_back(entity);
    }

    AINFO << "Converted TransformStamped to SceneUpdate: "
          << message.header().frame_id() << " -> " << message.child_frame_id();

    return scene_update;
}

foxglove::schemas::SceneUpdate ToFoxgloveImpl(
    const autonomy::commsgs::proto::geometry_msgs::TransformStampeds& message) {
    foxglove::schemas::SceneUpdate scene_update;

    if (message.transforms_size() == 0) {
        AWARN << "TransformStampeds is empty, returning empty SceneUpdate";
        return scene_update;
    }

    // 遍历所有 transforms，为每个 transform 创建一个场景实体
    for (const auto& transform_stamped : message.transforms()) {
        // 创建场景实体
        foxglove::schemas::SceneEntity entity;
        SetEntityHeader(entity, ExtractTimestamp(transform_stamped),
                        ExtractFrameId(transform_stamped));

        entity.id = "transform_" + transform_stamped.child_frame_id();
        entity.frame_locked = false;

        // 显示坐标变换（使用箭头）
        if (transform_stamped.has_transform()) {
            const auto& transform = transform_stamped.transform();

            // 创建 Pose（从 Transform 转换）
            autonomy::commsgs::proto::geometry_msgs::Pose pose;
            pose.mutable_position()->set_x(transform.translation().x());
            pose.mutable_position()->set_y(transform.translation().y());
            pose.mutable_position()->set_z(transform.translation().z());
            pose.mutable_orientation()->set_x(transform.rotation().x());
            pose.mutable_orientation()->set_y(transform.rotation().y());
            pose.mutable_orientation()->set_z(transform.rotation().z());
            pose.mutable_orientation()->set_w(transform.rotation().w());

            foxglove::schemas::ArrowPrimitive arrow;
            arrow.pose = CreatePose(pose);

            arrow.shaft_length = 0.4;
            arrow.head_length = 0.1;
            arrow.shaft_diameter = 0.04;
            arrow.head_diameter = 0.12;

            // 设置颜色（紫色）
            arrow.color = CreateColor(1.0, 0.0, 1.0, 1.0);

            entity.arrows.push_back(arrow);
        }

        if (!entity.arrows.empty()) {
            scene_update.entities.push_back(entity);
        }
    }

    AINFO << "Converted TransformStampeds with " << message.transforms_size()
          << " transforms to SceneUpdate";

    return scene_update;
}

}  // namespace impl
}  // namespace converter
}  // namespace visualization
}  // namespace autonomy

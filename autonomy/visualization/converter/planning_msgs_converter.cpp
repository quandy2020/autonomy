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

#include "autonomy/visualization/converter/planning_msgs_converter.hpp"

#include <cmath>

#include "autolink/autolink.hpp"
#include "autonomy/visualization/converter/converter_detail.hpp"

namespace autonomy {
namespace visualization {
namespace converter {
namespace impl {

using detail::CreateColor;
using detail::ExtractFrameId;
using detail::ExtractTimestamp;
using detail::SetEntityHeader;

foxglove::schemas::SceneUpdate ToFoxgloveImpl(
    const autonomy::commsgs::proto::planning_msgs::Path& message) {
    foxglove::schemas::SceneUpdate scene_update;

    if (message.poses_size() == 0) {
        AWARN << "Path is empty, returning empty SceneUpdate";
        return scene_update;
    }

    // 创建场景实体
    foxglove::schemas::SceneEntity entity;
    SetEntityHeader(entity, ExtractTimestamp(message), ExtractFrameId(message));

    entity.id = "path";
    entity.frame_locked = false;

    // 创建线条 primitive 来显示路径
    foxglove::schemas::LinePrimitive line;
    line.type = foxglove::schemas::LinePrimitive::LineType::LINE_STRIP;
    line.thickness = 0.1;
    line.scale_invariant = false;

    line.color = CreateColor(0.0, 0.0, 1.0, 0.8);

    // 将路径点转换为线条点
    for (const auto& pose_stamped : message.poses()) {
        foxglove::schemas::Point3 point;
        point.x = pose_stamped.pose().position().x();
        point.y = pose_stamped.pose().position().y();
        point.z = pose_stamped.pose().position().z();
        line.points.push_back(point);
    }

    entity.lines.push_back(line);
    scene_update.entities.push_back(entity);

    AINFO << "Converted Path with " << message.poses_size()
          << " poses to SceneUpdate";

    return scene_update;
}

foxglove::schemas::SceneUpdate ToFoxgloveImpl(
    const autonomy::commsgs::proto::planning_msgs::Odometry& message) {
    foxglove::schemas::SceneUpdate scene_update;

    // 创建场景实体
    foxglove::schemas::SceneEntity entity;
    SetEntityHeader(entity, ExtractTimestamp(message), ExtractFrameId(message));

    entity.id = "odometry";
    entity.frame_locked = false;

    // 显示位置（使用立方体）
    if (message.has_pose() && message.pose().has_pose()) {
        const auto& pose = message.pose().pose();

        foxglove::schemas::CubePrimitive cube;
        cube.pose = foxglove::schemas::Pose();
        cube.pose->position = foxglove::schemas::Vector3();
        cube.pose->position->x = pose.position().x();
        cube.pose->position->y = pose.position().y();
        cube.pose->position->z = pose.position().z();

        cube.pose->orientation = foxglove::schemas::Quaternion();
        cube.pose->orientation->x = pose.orientation().x();
        cube.pose->orientation->y = pose.orientation().y();
        cube.pose->orientation->z = pose.orientation().z();
        cube.pose->orientation->w = pose.orientation().w();

        cube.size = foxglove::schemas::Vector3();
        cube.size->x = 0.3;
        cube.size->y = 0.3;
        cube.size->z = 0.3;

        cube.color = CreateColor(0.0, 1.0, 0.0, 0.6);
        entity.cubes.push_back(cube);
    }

    // 显示速度向量（使用箭头）
    if (message.has_twist() && message.twist().has_twist()) {
        const auto& twist = message.twist().twist();

        // 线性速度箭头
        if (twist.has_linear()) {
            double vx = twist.linear().x();
            double vy = twist.linear().y();
            double vz = twist.linear().z();
            double speed = sqrt(vx * vx + vy * vy + vz * vz);

            if (speed > 0.01) {
                foxglove::schemas::ArrowPrimitive vel_arrow;
                vel_arrow.pose = foxglove::schemas::Pose();

                if (message.has_pose() && message.pose().has_pose()) {
                    vel_arrow.pose->position = foxglove::schemas::Vector3();
                    vel_arrow.pose->position->x =
                        message.pose().pose().position().x();
                    vel_arrow.pose->position->y =
                        message.pose().pose().position().y();
                    vel_arrow.pose->position->z =
                        message.pose().pose().position().z();
                } else {
                    vel_arrow.pose->position = foxglove::schemas::Vector3();
                    vel_arrow.pose->position->x = 0.0;
                    vel_arrow.pose->position->y = 0.0;
                    vel_arrow.pose->position->z = 0.0;
                }

                vel_arrow.pose->orientation = foxglove::schemas::Quaternion();
                vel_arrow.pose->orientation->x = 0.0;
                vel_arrow.pose->orientation->y = 0.0;
                vel_arrow.pose->orientation->z = 0.0;
                vel_arrow.pose->orientation->w = 1.0;

                vel_arrow.shaft_length = speed * 0.5;
                vel_arrow.head_length = vel_arrow.shaft_length * 0.2;
                vel_arrow.shaft_diameter = 0.03;
                vel_arrow.head_diameter = 0.08;

                vel_arrow.color = CreateColor(1.0, 0.0, 0.0, 1.0);
                entity.arrows.push_back(vel_arrow);
            }
        }

        // 角速度箭头
        if (twist.has_angular()) {
            double wx = twist.angular().x();
            double wy = twist.angular().y();
            double wz = twist.angular().z();
            double ang_speed = sqrt(wx * wx + wy * wy + wz * wz);

            if (ang_speed > 0.01) {
                foxglove::schemas::ArrowPrimitive ang_arrow;
                ang_arrow.pose = foxglove::schemas::Pose();

                if (message.has_pose() && message.pose().has_pose()) {
                    ang_arrow.pose->position = foxglove::schemas::Vector3();
                    ang_arrow.pose->position->x =
                        message.pose().pose().position().x();
                    ang_arrow.pose->position->y =
                        message.pose().pose().position().y();
                    ang_arrow.pose->position->z =
                        message.pose().pose().position().z() + 0.5;
                } else {
                    ang_arrow.pose->position = foxglove::schemas::Vector3();
                    ang_arrow.pose->position->x = 0.0;
                    ang_arrow.pose->position->y = 0.0;
                    ang_arrow.pose->position->z = 0.5;
                }

                ang_arrow.pose->orientation = foxglove::schemas::Quaternion();
                ang_arrow.pose->orientation->x = 0.0;
                ang_arrow.pose->orientation->y = 0.0;
                ang_arrow.pose->orientation->z = 0.0;
                ang_arrow.pose->orientation->w = 1.0;

                ang_arrow.shaft_length = ang_speed * 0.5;
                ang_arrow.head_length = ang_arrow.shaft_length * 0.2;
                ang_arrow.shaft_diameter = 0.03;
                ang_arrow.head_diameter = 0.08;

                ang_arrow.color = CreateColor(0.0, 0.0, 1.0, 1.0);
                entity.arrows.push_back(ang_arrow);
            }
        }
    }

    if (!entity.cubes.empty() || !entity.arrows.empty()) {
        scene_update.entities.push_back(entity);
    }

    AINFO << "Converted Odometry to SceneUpdate with " << entity.cubes.size()
          << " cubes and " << entity.arrows.size() << " arrows";

    return scene_update;
}

}  // namespace impl
}  // namespace converter
}  // namespace visualization
}  // namespace autonomy

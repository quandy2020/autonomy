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

#include <cmath>

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

namespace {

foxglove::schemas::Quaternion QuaternionFromYaw(double yaw) {
  foxglove::schemas::Quaternion q;
  const double half = 0.5 * yaw;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}

// 将 Arrow 的 +X 轴对齐到给定向量（简化版：优先用 XY 平面的 yaw；纯 Z 方向做特判）
foxglove::schemas::Quaternion QuaternionFromVector(double x, double y, double z) {
  constexpr double kEps = 1e-9;
  if (std::abs(x) > kEps || std::abs(y) > kEps) {
    return QuaternionFromYaw(std::atan2(y, x));
  }

  // 近似纯 Z 方向：把 +X 旋转到 +/-Z（绕 Y 轴转 90deg）
  foxglove::schemas::Quaternion q;
  const double s = std::sqrt(0.5);  // sin(pi/4)=cos(pi/4)=sqrt(1/2)
  q.x = 0.0;
  q.z = 0.0;
  q.w = s;
  q.y = (z >= 0.0) ? -s : s;
  return q;
}

foxglove::schemas::Pose MakeArrowPose(double px, double py, double pz, double vx, double vy, double vz) {
  foxglove::schemas::Pose pose;
  pose.position = foxglove::schemas::Vector3();
  pose.position->x = px;
  pose.position->y = py;
  pose.position->z = pz;
  pose.orientation = QuaternionFromVector(vx, vy, vz);
  return pose;
}

}  // namespace

foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::geometry_msgs::PoseStamped& message) {
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

foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::geometry_msgs::PoseArray& message) {
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

  AINFO << "Converted PoseArray with " << message.poses_size() << " poses to SceneUpdate";

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

  AINFO << "Converted TransformStamped to SceneUpdate: " << message.header().frame_id() << " -> "
        << message.child_frame_id();

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
    SetEntityHeader(entity, ExtractTimestamp(transform_stamped), ExtractFrameId(transform_stamped));

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

  AINFO << "Converted TransformStampeds with " << message.transforms_size() << " transforms to SceneUpdate";

  return scene_update;
}

foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::geometry_msgs::Twist& message) {
  foxglove::schemas::SceneUpdate scene_update;

  foxglove::schemas::SceneEntity entity;
  SetEntityHeader(entity, /*timestamp=*/std::nullopt, /*frame_id=*/std::nullopt);
  entity.id = "twist";
  entity.frame_locked = false;

  // 线速度箭头（红色）
  if (message.has_linear()) {
    const double vx = message.linear().x();
    const double vy = message.linear().y();
    const double vz = message.linear().z();
    const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);

    if (speed > 0.01) {
      foxglove::schemas::ArrowPrimitive vel_arrow;
      // Foxglove 的 ArrowPrimitive 默认沿 pose 的 +X 轴延伸，所以用 pose.orientation 表达方向
      vel_arrow.pose = MakeArrowPose(/*px=*/0.0, /*py=*/0.0, /*pz=*/0.0, vx, vy, vz);

      vel_arrow.shaft_length = speed * 0.5;
      vel_arrow.head_length = vel_arrow.shaft_length * 0.2;
      vel_arrow.shaft_diameter = 0.03;
      vel_arrow.head_diameter = 0.08;
      vel_arrow.color = CreateColor(1.0, 0.0, 0.0, 1.0);

      entity.arrows.push_back(vel_arrow);
    }
  }

  // 角速度箭头（蓝色）
  if (message.has_angular()) {
    const double wx = message.angular().x();
    const double wy = message.angular().y();
    const double wz = message.angular().z();
    const double ang_speed = std::sqrt(wx * wx + wy * wy + wz * wz);

    if (ang_speed > 0.01) {
      foxglove::schemas::ArrowPrimitive ang_arrow;
      // 用角速度向量方向作为箭头方向（常见 2D 场景就是 +/-Z）
      ang_arrow.pose = MakeArrowPose(/*px=*/0.0, /*py=*/0.0, /*pz=*/0.5, wx, wy, wz);

      ang_arrow.shaft_length = ang_speed * 0.5;
      ang_arrow.head_length = ang_arrow.shaft_length * 0.2;
      ang_arrow.shaft_diameter = 0.03;
      ang_arrow.head_diameter = 0.08;
      ang_arrow.color = CreateColor(0.0, 0.0, 1.0, 1.0);

      entity.arrows.push_back(ang_arrow);
    }
  }

  if (!entity.arrows.empty()) {
    scene_update.entities.push_back(entity);
  }

  AINFO << "Converted Twist to SceneUpdate (arrows=" << entity.arrows.size() << ")";
  return scene_update;
}

foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::geometry_msgs::TwistStamped& message) {
  foxglove::schemas::SceneUpdate scene_update;

  foxglove::schemas::SceneEntity entity;
  SetEntityHeader(entity, ExtractTimestamp(message), ExtractFrameId(message));
  entity.id = "twist_stamped";
  entity.frame_locked = false;

  if (message.has_twist()) {
    const auto& twist = message.twist();

    // 线速度箭头（红色）
    if (twist.has_linear()) {
      const double vx = twist.linear().x();
      const double vy = twist.linear().y();
      const double vz = twist.linear().z();
      const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);

      if (speed > 0.01) {
        foxglove::schemas::ArrowPrimitive vel_arrow;
        vel_arrow.pose = MakeArrowPose(/*px=*/0.0, /*py=*/0.0, /*pz=*/0.0, vx, vy, vz);

        vel_arrow.shaft_length = speed * 0.5;
        vel_arrow.head_length = vel_arrow.shaft_length * 0.2;
        vel_arrow.shaft_diameter = 0.03;
        vel_arrow.head_diameter = 0.08;
        vel_arrow.color = CreateColor(1.0, 0.0, 0.0, 1.0);

        entity.arrows.push_back(vel_arrow);
      }
    }

    // 角速度箭头（蓝色）
    if (twist.has_angular()) {
      const double wx = twist.angular().x();
      const double wy = twist.angular().y();
      const double wz = twist.angular().z();
      const double ang_speed = std::sqrt(wx * wx + wy * wy + wz * wz);

      if (ang_speed > 0.01) {
        foxglove::schemas::ArrowPrimitive ang_arrow;
        ang_arrow.pose = MakeArrowPose(/*px=*/0.0, /*py=*/0.0, /*pz=*/0.5, wx, wy, wz);

        ang_arrow.shaft_length = ang_speed * 0.5;
        ang_arrow.head_length = ang_arrow.shaft_length * 0.2;
        ang_arrow.shaft_diameter = 0.03;
        ang_arrow.head_diameter = 0.08;
        ang_arrow.color = CreateColor(0.0, 0.0, 1.0, 1.0);

        entity.arrows.push_back(ang_arrow);
      }
    }
  }

  if (!entity.arrows.empty()) {
    scene_update.entities.push_back(entity);
  }

  AINFO << "Converted TwistStamped to SceneUpdate (arrows=" << entity.arrows.size() << ")";
  return scene_update;
}

}  // namespace impl
}  // namespace converter
}  // namespace visualization
}  // namespace autonomy

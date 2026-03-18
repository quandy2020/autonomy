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

#include "autonomy/visualization/converter/visualization_msgs_converter.hpp"

#include <string>

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

foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::visualization_msgs::Marker& message) {
  foxglove::schemas::SceneUpdate scene_update;

  // 处理 DELETE 和 DELETEALL 操作
  if (message.action() == 2) {  // DELETE
    foxglove::schemas::SceneEntity entity;
    entity.id = message.ns() + "_" + std::to_string(message.id());
    scene_update.entities.push_back(entity);
    return scene_update;
  } else if (message.action() == 3) {  // DELETEALL
    AINFO << "DELETEALL action received, should be handled at higher level";
    return scene_update;
  }

  // 创建场景实体
  foxglove::schemas::SceneEntity entity;
  SetEntityHeader(entity, ExtractTimestamp(message), ExtractFrameId(message));

  entity.id = message.ns() + "_" + std::to_string(message.id());
  entity.frame_locked = message.frame_locked();

  // 根据 Marker 类型创建不同的 primitive
  switch (message.type()) {
    case 0: {  // ARROW
      if (message.has_pose()) {
        foxglove::schemas::ArrowPrimitive arrow;
        arrow.pose = CreatePose(message.pose());

        if (message.has_scale()) {
          arrow.shaft_length = message.scale().x();
          arrow.head_length = message.scale().y();
          arrow.shaft_diameter = message.scale().z();
          arrow.head_diameter = message.scale().z() * 2.0;
        } else {
          arrow.shaft_length = 0.5;
          arrow.head_length = 0.1;
          arrow.shaft_diameter = 0.05;
          arrow.head_diameter = 0.15;
        }

        if (message.has_color()) {
          arrow.color = foxglove::schemas::Color();
          arrow.color->r = message.color().r();
          arrow.color->g = message.color().g();
          arrow.color->b = message.color().b();
          arrow.color->a = message.color().a();
        }

        entity.arrows.push_back(arrow);
      }
      break;
    }
    case 1: {  // CUBE
      if (message.has_pose()) {
        foxglove::schemas::CubePrimitive cube;
        cube.pose = CreatePose(message.pose());

        if (message.has_scale()) {
          cube.size = foxglove::schemas::Vector3();
          cube.size->x = message.scale().x();
          cube.size->y = message.scale().y();
          cube.size->z = message.scale().z();
        } else {
          cube.size = foxglove::schemas::Vector3();
          cube.size->x = 1.0;
          cube.size->y = 1.0;
          cube.size->z = 1.0;
        }

        if (message.has_color()) {
          cube.color = foxglove::schemas::Color();
          cube.color->r = message.color().r();
          cube.color->g = message.color().g();
          cube.color->b = message.color().b();
          cube.color->a = message.color().a();
        }

        entity.cubes.push_back(cube);
      }
      break;
    }
    case 2: {  // SPHERE
      if (message.has_pose()) {
        foxglove::schemas::SpherePrimitive sphere;
        sphere.pose = CreatePose(message.pose());

        if (message.has_scale()) {
          double radius = message.scale().x() / 2.0;
          sphere.size = foxglove::schemas::Vector3();
          sphere.size->x = radius * 2.0;
          sphere.size->y = radius * 2.0;
          sphere.size->z = radius * 2.0;
        } else {
          sphere.size = foxglove::schemas::Vector3();
          sphere.size->x = 1.0;
          sphere.size->y = 1.0;
          sphere.size->z = 1.0;
        }

        if (message.has_color()) {
          sphere.color = foxglove::schemas::Color();
          sphere.color->r = message.color().r();
          sphere.color->g = message.color().g();
          sphere.color->b = message.color().b();
          sphere.color->a = message.color().a();
        }

        entity.spheres.push_back(sphere);
      }
      break;
    }
    case 3: {  // CYLINDER
      if (message.has_pose()) {
        foxglove::schemas::CylinderPrimitive cylinder;
        cylinder.pose = CreatePose(message.pose());

        if (message.has_scale()) {
          cylinder.size = foxglove::schemas::Vector3();
          cylinder.size->x = message.scale().x();
          cylinder.size->y = message.scale().y();
          cylinder.size->z = message.scale().y();
        } else {
          cylinder.size = foxglove::schemas::Vector3();
          cylinder.size->x = 1.0;
          cylinder.size->y = 0.5;
          cylinder.size->z = 0.5;
        }

        if (message.has_color()) {
          cylinder.color = foxglove::schemas::Color();
          cylinder.color->r = message.color().r();
          cylinder.color->g = message.color().g();
          cylinder.color->b = message.color().b();
          cylinder.color->a = message.color().a();
        }

        entity.cylinders.push_back(cylinder);
      }
      break;
    }
    case 4:    // LINE_STRIP
    case 5: {  // LINE_LIST
      if (message.points_size() > 0) {
        foxglove::schemas::LinePrimitive line;
        line.type = (message.type() == 4) ? foxglove::schemas::LinePrimitive::LineType::LINE_STRIP
                                          : foxglove::schemas::LinePrimitive::LineType::LINE_LIST;

        if (message.has_scale()) {
          line.thickness = message.scale().x();
        } else {
          line.thickness = 0.05;
        }
        line.scale_invariant = false;

        for (const auto& point : message.points()) {
          foxglove::schemas::Point3 p;
          p.x = point.x();
          p.y = point.y();
          p.z = point.z();
          line.points.push_back(p);
        }

        if (message.has_color()) {
          line.color = foxglove::schemas::Color();
          line.color->r = message.color().r();
          line.color->g = message.color().g();
          line.color->b = message.color().b();
          line.color->a = message.color().a();
        } else {
          line.color = CreateColor(1.0, 1.0, 1.0, 1.0);
        }

        entity.lines.push_back(line);
      }
      break;
    }
    case 8: {  // POINTS
      if (message.points_size() > 0) {
        foxglove::schemas::LinePrimitive points_line;
        points_line.type = foxglove::schemas::LinePrimitive::LineType::LINE_LIST;

        double point_size = 0.1;
        if (message.has_scale()) {
          point_size = message.scale().x();
        }
        points_line.thickness = point_size;
        points_line.scale_invariant = false;

        for (const auto& point : message.points()) {
          foxglove::schemas::Point3 p1;
          p1.x = point.x();
          p1.y = point.y();
          p1.z = point.z();
          points_line.points.push_back(p1);

          foxglove::schemas::Point3 p2;
          p2.x = point.x() + point_size * 0.01;
          p2.y = point.y();
          p2.z = point.z();
          points_line.points.push_back(p2);
        }

        if (message.has_color()) {
          points_line.color = foxglove::schemas::Color();
          points_line.color->r = message.color().r();
          points_line.color->g = message.color().g();
          points_line.color->b = message.color().b();
          points_line.color->a = message.color().a();
        } else {
          points_line.color = CreateColor(1.0, 1.0, 1.0, 1.0);
        }

        entity.lines.push_back(points_line);
      }
      break;
    }
    default:
      AWARN << "Unsupported Marker type: " << message.type();
      break;
  }

  if (!entity.arrows.empty() || !entity.cubes.empty() || !entity.spheres.empty() || !entity.cylinders.empty() ||
      !entity.lines.empty()) {
    scene_update.entities.push_back(entity);
  }

  AINFO << "Converted Marker (type=" << message.type() << ", id=" << message.id() << ") to SceneUpdate";

  return scene_update;
}

foxglove::schemas::SceneUpdate ToFoxgloveImpl(
    const autonomy::commsgs::proto::visualization_msgs::MarkerArray& message) {
  foxglove::schemas::SceneUpdate scene_update;

  if (message.markers_size() == 0) {
    AWARN << "MarkerArray is empty, returning empty SceneUpdate";
    return scene_update;
  }

  // 将每个 Marker 转换为 SceneEntity 并添加到 SceneUpdate
  for (const auto& marker : message.markers()) {
    auto marker_scene = ToFoxgloveImpl(marker);
    for (const auto& entity : marker_scene.entities) {
      scene_update.entities.push_back(entity);
    }
  }

  AINFO << "Converted MarkerArray with " << message.markers_size() << " markers to SceneUpdate";

  return scene_update;
}

}  // namespace impl
}  // namespace converter
}  // namespace visualization
}  // namespace autonomy

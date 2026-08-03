/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/strata_semantic_zone_display.hpp"

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/display/ogre_colored_points_draw.hpp"
#include "autoviz/display/ogre_label_draw.hpp"
#include "autoviz/display/ogre_overlay_draw.hpp"
#include "autoviz/display/obj_mesh.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {
namespace {

QColor ProtoColor(const automsgs::msgs::std_msgs::ColorRGBA& color,
                  float alpha_override = -1.f) {
  QColor result(static_cast<int>(color.r() * 255.f),
                static_cast<int>(color.g() * 255.f),
                static_cast<int>(color.b() * 255.f));
  const float alpha = alpha_override >= 0.f ? alpha_override : color.a();
  result.setAlphaF(alpha);
  return result;
}

QVector3D PolygonCentroid(const std::vector<QVector3D>& polygon) {
  if (polygon.empty()) {
    return {};
  }
  QVector3D sum;
  for (const auto& point : polygon) {
    sum += point;
  }
  return sum / static_cast<float>(polygon.size());
}

ObjMesh BuildFanMesh(const std::vector<QVector3D>& polygon, const QVector3D& centroid) {
  ObjMesh mesh;
  if (polygon.size() < 3) {
    return mesh;
  }
  mesh.vertices.push_back(centroid);
  for (const auto& point : polygon) {
    if (mesh.vertices.size() <= 1 ||
        (point - mesh.vertices.back()).lengthSquared() > 1e-8f) {
      mesh.vertices.push_back(point);
    }
  }
  if (mesh.vertices.size() < 4) {
    return {};
  }
  for (size_t i = 1; i + 1 < mesh.vertices.size(); ++i) {
    mesh.triangles.push_back(
        {0, static_cast<int>(i), static_cast<int>(i + 1)});
  }
  return mesh;
}

std::vector<float> DashPatternForZoneType(const std::string& zone_type) {
  if (zone_type == "forbidden") {
    return {4.f, 3.f};
  }
  if (zone_type == "speed_limit") {
    return {6.f, 3.f};
  }
  if (zone_type == "elevator") {
    return {3.f, 3.f};
  }
  if (zone_type == "waiting") {
    return {5.f, 3.f};
  }
  if (zone_type == "activity_area") {
    return {8.f, 4.f};
  }
  return {};
}

void DrawDashedLoop(rendering::SceneOverlay& scene, const std::vector<QVector3D>& polygon,
                    const QColor& color, const std::vector<float>& dash_pattern) {
  if (polygon.size() < 2) {
    return;
  }
  std::vector<QVector3D> loop = polygon;
  if ((loop.front() - loop.back()).lengthSquared() > 1e-8f) {
    loop.push_back(loop.front());
  }
  const float dash_on = dash_pattern.empty() ? 0.f : dash_pattern[0] * 0.01f;
  const float dash_off = dash_pattern.size() > 1 ? dash_pattern[1] * 0.01f : dash_on;
  if (dash_on <= 0.f) {
    for (size_t i = 0; i + 1 < loop.size(); ++i) {
      scene.addLine(loop[i], loop[i + 1], color);
    }
    return;
  }

  for (size_t i = 0; i + 1 < loop.size(); ++i) {
    const QVector3D start = loop[i];
    const QVector3D end = loop[i + 1];
    const QVector3D delta = end - start;
    const float length = delta.length();
    if (length <= 1e-6f) {
      continue;
    }
    const QVector3D direction = delta / length;
    float traveled = 0.f;
    bool draw_segment = true;
    while (traveled < length) {
      const float span = draw_segment ? dash_on : dash_off;
      const float next = std::min(traveled + span, length);
      if (draw_segment) {
        scene.addLine(start + direction * traveled, start + direction * next, color);
      }
      traveled = next;
      draw_segment = !draw_segment;
    }
  }
}

}  // namespace

StrataSemanticZoneDisplay::StrataSemanticZoneDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::strata_msgs::SemanticZoneArray>(
          "StrataSemanticZone", std::move(channel),
          "automsgs.msgs.strata_msgs.SemanticZoneArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> StrataSemanticZoneDisplay::propertySpecs()
    const {
  using common::DisplayPropertyKind;
  return {{"fill_color", "Fill Color", "100;149;237", {}, DisplayPropertyKind::kColor},
          {"outline_color", "Outline Color", "65;105;225", {}, DisplayPropertyKind::kColor},
          {"label_color", "Label Color", "255;255;255", {}, DisplayPropertyKind::kColor},
          {"use_custom_colors", "Use Custom Colors", "false"},
          {"show_fill", "Show Fill", "true"},
          {"show_outline", "Show Outline", "true"},
          {"show_labels", "Show Labels", "true"}};
}

void StrataSemanticZoneDisplay::onPropertyChanged(const std::string& /*key*/) {
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void StrataSemanticZoneDisplay::processMessage(
    const automsgs::msgs::strata_msgs::SemanticZoneArray& message) {
  zones_.clear();
  if (context_ == nullptr) {
    return;
  }

  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();

  for (const auto& zone : message.zones()) {
    if (zone.polygon_size() < 3) {
      continue;
    }
    StoredZone stored;
    stored.outline_width = zone.outline_width() > 0.f ? zone.outline_width() * 0.01f
                                                      : 0.02f;
    stored.zone_type = zone.zone_type();
    stored.dash_pattern = DashPatternForZoneType(stored.zone_type);
    stored.fill_color = ProtoColor(zone.fill_color(), zone.fill_opacity());
    stored.outline_color = ProtoColor(zone.outline_color(), zone.fill_opacity());
    if (!zone.label().empty()) {
      stored.label = QString::fromStdString(zone.label());
    } else {
      stored.label = QString::fromStdString(zone.zone_type());
    }

    std::vector<QVector3D> local_polygon;
    local_polygon.reserve(static_cast<size_t>(zone.polygon_size()));
    bool transform_failed = false;
    for (const auto& point : zone.polygon()) {
      QVector3D local(static_cast<float>(point.x()), static_cast<float>(point.y()),
                      static_cast<float>(point.z()) + 0.01f);
      if (frame != context_->fixed_frame) {
        try {
          const auto tf = context_->tf_buffer->lookupTransform(
              context_->fixed_frame, frame, zero_time);
          local = transformPoint(tf, local);
        } catch (...) {
          transform_failed = true;
          break;
        }
      }
      local_polygon.push_back(local);
    }
    if (transform_failed || local_polygon.size() < 3) {
      continue;
    }
    stored.polygon = std::move(local_polygon);
    zones_.push_back(std::move(stored));
  }

  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void StrataSemanticZoneDisplay::DrawZoneFill(rendering::SceneOverlay& scene,
                                             const StoredZone& zone) const {
  if (zone.polygon.size() < 3) {
    return;
  }
  const QVector3D centroid = PolygonCentroid(zone.polygon);
  const ObjMesh mesh = BuildFanMesh(zone.polygon, centroid);
  if (!mesh.vertices.empty()) {
    scene.addTriangleMeshSolid(mesh, {}, zone.fill_color);
  }
}

void StrataSemanticZoneDisplay::DrawZoneOutline(rendering::SceneOverlay& scene,
                                                const StoredZone& zone) const {
  if (zone.polygon.size() < 2) {
    return;
  }
  if (!zone.dash_pattern.empty()) {
    DrawDashedLoop(scene, zone.polygon, zone.outline_color, zone.dash_pattern);
    return;
  }
  if (zone.outline_width > 0.f) {
    drawBillboardStripOgreOrGl(context_, scene, name() + "/outline", zone.polygon,
                               zone.outline_color, zone.outline_width);
    return;
  }
  for (size_t i = 0; i + 1 < zone.polygon.size(); ++i) {
    scene.addLine(zone.polygon[i], zone.polygon[i + 1], zone.outline_color);
  }
  scene.addLine(zone.polygon.back(), zone.polygon.front(), zone.outline_color);
}

void StrataSemanticZoneDisplay::onDraw(rendering::SceneOverlay& scene) {
  const bool show_fill =
      common::ParseBoolProperty(propertyValue("show_fill", "true"), true);
  const bool show_outline =
      common::ParseBoolProperty(propertyValue("show_outline", "true"), true);
  const bool show_labels =
      common::ParseBoolProperty(propertyValue("show_labels", "true"), true);
  const bool use_custom_colors =
      common::ParseBoolProperty(propertyValue("use_custom_colors", "false"), false);
  const QColor custom_fill = common::ParseColorProperty(
      propertyValue("fill_color", "100;149;237"), QColor(100, 149, 237));
  const QColor custom_outline = common::ParseColorProperty(
      propertyValue("outline_color", "65;105;225"), QColor(65, 105, 225));
  const QColor custom_label = common::ParseColorProperty(
      propertyValue("label_color", "255;255;255"), QColor(255, 255, 255));

  for (size_t i = 0; i < zones_.size(); ++i) {
    StoredZone zone = zones_[i];
    if (use_custom_colors) {
      zone.fill_color = custom_fill;
      zone.fill_color.setAlphaF(zones_[i].fill_color.alphaF());
      zone.outline_color = custom_outline;
      zone.outline_color.setAlphaF(zones_[i].outline_color.alphaF());
    }
    if (show_fill) {
      DrawZoneFill(scene, zone);
    }
    if (show_outline) {
      DrawZoneOutline(scene, zone);
    }
    if (show_labels && !zone.label.isEmpty()) {
      const QVector3D centroid = PolygonCentroid(zone.polygon);
      TextLabelInstance label;
      label.text = zone.label.toStdString();
      *label.mutable_position() = centroid + QVector3D(0.f, 0.f, 0.03f);
      *label.mutable_color() = use_custom_colors ? custom_label : zone.outline_color;
      label.char_height = 0.1f;
      drawLabelsOgreOrGl(context_, scene, name() + "/label/" + std::to_string(i), {label});
    }
  }
}

}  // namespace display
}  // namespace autoviz

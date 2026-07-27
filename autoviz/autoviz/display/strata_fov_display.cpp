/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/strata_fov_display.hpp"

#include <array>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/display/obj_mesh.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {
namespace {

constexpr int32_t kMarkerLineStrip = 4;

QColor ProtoColor(const automsgs::msgs::std_msgs::ColorRGBA& color) {
  return QColor(static_cast<int>(color.r() * 255.f),
                static_cast<int>(color.g() * 255.f),
                static_cast<int>(color.b() * 255.f),
                static_cast<int>(color.a() * 255.f));
}

ObjMesh BuildFanMesh(const std::vector<QVector3D>& polygon, const QVector3D& centroid) {
  ObjMesh mesh;
  if (polygon.size() < 3) {
    return mesh;
  }
  mesh.vertices.push_back(centroid);
  for (const auto& point : polygon) {
    if (mesh.vertices.empty() ||
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

}  // namespace

StrataFovDisplay::StrataFovDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::visualization_msgs::MarkerArray>(
          "StrataFov", std::move(channel),
          "autonomy.commsgs.proto.visualization_msgs.MarkerArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> StrataFovDisplay::propertySpecs() const {
  using common::DisplayPropertyKind;
  return {{"fill_color", "Fill Color", "0;102;255", {}, DisplayPropertyKind::kColor},
          {"use_custom_color", "Use Custom Color", "false"},
          {"show_fill", "Show Fill", "true"},
          {"show_outline", "Show Outline", "true"}};
}

void StrataFovDisplay::onPropertyChanged(const std::string& /*key*/) {
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void StrataFovDisplay::processMessage(
    const automsgs::msgs::visualization_msgs::MarkerArray& message) {
  bands_.clear();
  if (context_ == nullptr) {
    return;
  }

  const auto zero_time = autoviz::commsgs::ZeroTime();
  for (const auto& marker : message.markers()) {
    if (marker.ns() != "strata_robot_fov" || marker.type() != kMarkerLineStrip ||
        marker.points_size() < 3) {
      continue;
    }

    const std::string frame = marker.header().frame_id().empty()
                                  ? context_->fixed_frame
                                  : marker.header().frame_id();
    StoredBand band;
    band.fill_color = ProtoColor(marker.color());
    band.polygon.reserve(static_cast<size_t>(marker.points_size()));
    bool transform_failed = false;
    for (const auto& point : marker.points()) {
      QVector3D local(static_cast<float>(point.x()), static_cast<float>(point.y()),
                      static_cast<float>(point.z()) + 0.005f);
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
      band.polygon.push_back(local);
    }
    if (!transform_failed && band.polygon.size() >= 3) {
      bands_.push_back(std::move(band));
    }
  }

  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void StrataFovDisplay::onDraw(rendering::SceneOverlay& scene) {
  const bool show_fill =
      common::ParseBoolProperty(propertyValue("show_fill", "true"), true);
  const bool show_outline =
      common::ParseBoolProperty(propertyValue("show_outline", "true"), true);
  const bool use_custom_color =
      common::ParseBoolProperty(propertyValue("use_custom_color", "false"), false);
  const QColor custom_color = common::ParseColorProperty(
      propertyValue("fill_color", "0;102;255"), QColor(0, 102, 255));

  for (size_t i = 0; i < bands_.size(); ++i) {
    const auto& band = bands_[i];
    if (band.polygon.size() < 3) {
      continue;
    }
    QColor draw_color = band.fill_color;
    if (use_custom_color) {
      draw_color = custom_color;
      draw_color.setAlphaF(band.fill_color.alphaF());
    }
    const QVector3D centroid = PolygonCentroid(band.polygon);
    if (show_fill) {
      const ObjMesh mesh = BuildFanMesh(band.polygon, centroid);
      if (!mesh.vertices.empty()) {
        scene.addTriangleMeshSolid(mesh, {}, draw_color);
      }
    }
    if (show_outline) {
      for (size_t j = 0; j + 1 < band.polygon.size(); ++j) {
        scene.addLine(band.polygon[j], band.polygon[j + 1], draw_color);
      }
    }
  }
}

}  // namespace display
}  // namespace autoviz

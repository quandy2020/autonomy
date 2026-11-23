/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/radar/radar_tracks_display.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

#include <QMatrix4x4>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/plugins/displays/common/color_category.hpp"
#include "autoviz/plugins/displays/common/transform_utils.hpp"
#include "autoviz/rendering/text_raster_utils.hpp"

namespace autoviz {
namespace display {
namespace {

constexpr float kPi = 3.14159265358979323846f;

std::uint32_t TrackIdFromUuid(
    const automsgs::msgs::unique_identifier_msgs::UUID& uuid, int fallback) {
  const std::string& bytes = uuid.uuid();
  if (bytes.size() >= 4) {
    const auto* p = reinterpret_cast<const unsigned char*>(bytes.data());
    return (static_cast<std::uint32_t>(p[0])) |
           (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) |
           (static_cast<std::uint32_t>(p[3]) << 24);
  }
  if (!bytes.empty()) {
    return static_cast<unsigned char>(bytes[0]);
  }
  return static_cast<std::uint32_t>(fallback);
}

QColor WithAlpha(QColor c, float alpha) {
  c.setAlphaF(std::clamp(alpha, 0.f, 1.f));
  return c;
}

void DrawGroundDisc(rendering::SceneOverlay& scene, const QVector3D& center,
                    float radius, const QColor& fill, const QColor& rim) {
  constexpr int kSegs = 24;
  const QVector3D origin(center.x(), center.y(), 0.05f);
  QVector3D prev = origin;
  std::vector<QVector3D> loop;
  loop.reserve(kSegs + 1);
  for (int i = 0; i <= kSegs; ++i) {
    const float t = 2.f * kPi * static_cast<float>(i) / static_cast<float>(kSegs);
    const QVector3D p(center.x() + radius * std::cos(t),
                      center.y() + radius * std::sin(t), 0.05f);
    if (i > 0) {
      scene.addColoredTriangle(origin, prev, p, fill, fill, fill);
    }
    loop.push_back(p);
    prev = p;
  }
  scene.addLineLoop(loop, rim, 1.5f);
}

void DrawArrowHead(rendering::SceneOverlay& scene, const QVector3D& from,
                   const QVector3D& to, const QColor& color) {
  QVector3D dir = to - from;
  const float len = dir.length();
  if (len < 1e-4f) {
    return;
  }
  dir /= len;
  QVector3D side(-dir.y(), dir.x(), 0.f);
  if (side.lengthSquared() < 1e-8f) {
    side = QVector3D(1.f, 0.f, 0.f);
  } else {
    side.normalize();
  }
  const float head = std::clamp(len * 0.28f, 0.08f, 0.22f);
  const QVector3D left = to - dir * head + side * head * 0.45f;
  const QVector3D right = to - dir * head - side * head * 0.45f;
  const QVector3D tip(to.x(), to.y(), 0.08f);
  scene.addColoredTriangle(tip, QVector3D(left.x(), left.y(), 0.08f),
                           QVector3D(right.x(), right.y(), 0.08f), color, color,
                           color);
}

void DrawBillboard(rendering::SceneOverlay& scene, const QVector3D& pos,
                   const QString& text, const QColor& color, float char_h) {
  const float h = std::max(0.12f, char_h);
  const int pixel_h = static_cast<int>(std::clamp(h * 180.f, 28.f, 160.f));
  const QImage img = rendering::RasterizeTextLabel(text, color, pixel_h);
  if (img.isNull()) {
    return;
  }
  const float aspect = static_cast<float>(img.width()) /
                       static_cast<float>(std::max(1, img.height()));
  scene.addViewFacingTexturedQuad(pos, h * 0.5f * aspect, h * 0.5f, img);
}

}  // namespace

RadarTracksDisplay::RadarTracksDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::sensor_msgs::RadarTracks>(
          "RadarTracks", std::move(channel),
          "automsgs.msgs.sensor_msgs.RadarTracks") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec> RadarTracksDisplay::propertySpecs()
    const {
  return {{"coloring", "Coloring", "Auto", {"Auto", "Flat", "Classification"}},
          {"color", "Color", "255;92;92", {},
           common::DisplayPropertyKind::kColor},
          {"label_color", "Label Color", "255;230;120", {},
           common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "0.88"},
          {"style", "Style", "Solid", {"Wireframe", "Solid"}},
          {"show_velocity", "Show Velocity", "true", {"true", "false"}},
          {"velocity_scale", "Velocity Scale", "0.6"},
          {"show_id", "Show ID", "true", {"true", "false"}},
          {"show_range", "Show Range", "true", {"true", "false"}},
          {"min_size", "Min Size (m)", "0.28"},
          {"label_height", "Label Height (m)", "0.28"}};
}

void RadarTracksDisplay::clearReceivedData() { tracks_.clear(); }

void RadarTracksDisplay::onPropertyChanged(const std::string& /*key*/) {
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void RadarTracksDisplay::processMessage(
    const automsgs::msgs::sensor_msgs::RadarTracks& message) {
  tracks_.clear();
  if (context_ == nullptr) {
    return;
  }

  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();
  automsgs::msgs::geometry_msgs::TransformStamped tf;
  bool have_tf = false;
  try {
    tf = context_->tf_buffer->lookupTransform(context_->fixed_frame, frame,
                                              zero_time);
    have_tf = true;
  } catch (...) {
  }

  const std::string coloring = propertyValue("coloring", "Auto");
  QColor flat = common::ParseColorProperty(propertyValue("color", "255;92;92"),
                                           QColor(255, 92, 92));
  QColor label_color = common::ParseColorProperty(
      propertyValue("label_color", "255;230;120"), QColor(255, 230, 120));
  const bool show_vel = propertyValue("show_velocity", "true") == "true";
  const bool show_id = propertyValue("show_id", "true") == "true";
  const bool show_range = propertyValue("show_range", "true") == "true";
  const float vel_scale = common::ParseFloatProperty(
      propertyValue("velocity_scale", "0.6"), 0.6f);
  const float min_size =
      common::ParseFloatProperty(propertyValue("min_size", "0.28"), 0.28f);

  tracks_.reserve(static_cast<std::size_t>(message.tracks_size()));
  for (int i = 0; i < message.tracks_size(); ++i) {
    const auto& track = message.tracks(i);
    TrackVisual visual;
    QVector3D local(static_cast<float>(track.position().x()),
                    static_cast<float>(track.position().y()),
                    static_cast<float>(track.position().z()));
    visual.center = have_tf ? transformPoint(tf, local) : local;

    const float sx = static_cast<float>(
        std::max(track.size().x(), static_cast<double>(min_size)));
    const float sy = static_cast<float>(
        std::max(track.size().y(), static_cast<double>(min_size)));
    const float sz = static_cast<float>(
        std::max(track.size().z(), static_cast<double>(min_size * 0.7)));
    visual.size = QVector3D(sx, sy, sz);

    QVector3D vel_local(static_cast<float>(track.velocity().x() * vel_scale),
                        static_cast<float>(track.velocity().y() * vel_scale),
                        static_cast<float>(track.velocity().z() * vel_scale));
    const QVector3D tip_local = local + vel_local;
    visual.velocity_tip = have_tf ? transformPoint(tf, tip_local) : tip_local;
    visual.draw_velocity =
        show_vel && (std::abs(track.velocity().x()) +
                     std::abs(track.velocity().y()) +
                     std::abs(track.velocity().z())) > 1e-4;

    if (coloring == "Flat") {
      visual.color = flat;
    } else if (coloring == "Classification") {
      visual.color = viz::ColorCategory20(
          static_cast<std::size_t>(track.classification()));
    } else {
      visual.color = viz::ColorCategory20(static_cast<std::size_t>(i));
    }
    visual.label_color = label_color;

    if (show_id || show_range) {
      const std::uint32_t tid = TrackIdFromUuid(track.uuid(), i) + 1;
      char buf[64];
      if (show_id && show_range) {
        const float range = std::hypot(local.x(), local.y());
        std::snprintf(buf, sizeof(buf), "ID %u  %.2fm", tid, range);
      } else if (show_id) {
        std::snprintf(buf, sizeof(buf), "ID %u", tid);
      } else {
        const float range = std::hypot(local.x(), local.y());
        std::snprintf(buf, sizeof(buf), "%.2fm", range);
      }
      visual.label = buf;
    }

    tracks_.push_back(visual);
  }

  setStatusOk();
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void RadarTracksDisplay::onDraw(rendering::SceneOverlay& scene) {
  const bool solid = propertyValue("style", "Solid") == "Solid";
  const float alpha = std::clamp(
      common::ParseFloatProperty(propertyValue("alpha", "0.88"), 0.88f), 0.f,
      1.f);
  const float label_height =
      common::ParseFloatProperty(propertyValue("label_height", "0.28"), 0.28f);

  for (const TrackVisual& track : tracks_) {
    const QColor fill = WithAlpha(track.color, alpha);
    const float disc_r =
        0.5f * std::max(track.size.x(), track.size.y()) + 0.04f;
    DrawGroundDisc(scene, track.center, disc_r,
                   WithAlpha(track.color, alpha * 0.18f),
                   WithAlpha(track.color, alpha * 0.8f));

    QMatrix4x4 transform;
    transform.translate(track.center.x(), track.center.y(),
                        std::max(track.center.z(), track.size.z() * 0.5f));
    const QVector3D half = track.size * 0.5f;
    if (solid) {
      scene.addBoxSolid(QVector3D(0.f, 0.f, 0.f), half, transform, fill);
    } else {
      scene.addBoxWireframe(QVector3D(0.f, 0.f, 0.f), half, transform, fill);
    }
    if (track.draw_velocity) {
      const QVector3D from(track.center.x(), track.center.y(), 0.08f);
      const QVector3D to(track.velocity_tip.x(), track.velocity_tip.y(), 0.08f);
      scene.addLine(from, to, fill);
      DrawArrowHead(scene, from, to, fill);
    }
    if (!track.label.empty()) {
      DrawBillboard(scene,
                    track.center + QVector3D(0.f, 0.f, half.z() + 0.18f),
                    QString::fromStdString(track.label), track.label_color,
                    label_height);
    }
  }
}

}  // namespace display
}  // namespace autoviz

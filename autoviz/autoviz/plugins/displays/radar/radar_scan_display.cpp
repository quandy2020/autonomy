/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/radar/radar_scan_display.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include <QFont>
#include <QPainter>
#include <QPainterPath>

#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/common/ogre_colored_points_draw.hpp"
#include "autoviz/plugins/displays/common/scalar_sensor_utils.hpp"
#include "autoviz/plugins/displays/common/transform_utils.hpp"
#include "autoviz/plugins/displays/pointcloud/point_cloud_utils.hpp"
#include "autoviz/rendering/point_cloud_style_utils.hpp"
#include "autoviz/rendering/text_raster_utils.hpp"

namespace autoviz {
namespace display {
namespace {
constexpr float kRad2Deg = 57.2957795f;
constexpr float kPi = 3.14159265358979323846f;

QColor MixRangeColor(float range, float max_range) {
  const float t = std::clamp(range / std::max(max_range, 0.5f), 0.f, 1.f);
  // Near = cyan, far = amber.
  const int r = static_cast<int>(40 + 215 * t);
  const int g = static_cast<int>(230 - 70 * t);
  const int b = static_cast<int>(255 - 185 * t);
  return QColor(r, g, b);
}

QColor WithAlpha(QColor c, float alpha) {
  c.setAlphaF(std::clamp(alpha, 0.f, 1.f));
  return c;
}

}  // namespace

RadarScanDisplay::RadarScanDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::sensor_msgs::RadarScan>(
          "RadarScan", std::move(channel),
          "automsgs.msgs.sensor_msgs.RadarScan") {
  setProperties({});
}

RadarScanDisplay::~RadarScanDisplay() {
  if (hud_) {
    hud_->hide();
  }
}

std::vector<common::DisplayPropertySpec> RadarScanDisplay::propertySpecs()
    const {
  return {{"point_size", "Size (Pixels)", "28", {}},
          {"style", "Style", "Spheres",
           {"Points", "Squares", "Flat Squares", "Spheres", "Tiles", "Boxes"}},
          {"color_transform", "Color Transformer", "Range",
           {"Flat", "Range", "Amplitude", "Doppler"}},
          {"alpha", "Alpha", "1.0"},
          {"color", "Color", "255;255;255", {},
           common::DisplayPropertyKind::kColor},
          {"show_range", "Show Range Labels", "true", {"true", "false"}},
          {"show_spokes", "Show Range Spokes", "true", {"true", "false"}},
          {"show_halo", "Show Halo", "true", {"true", "false"}},
          {"show_hud", "Show Info Panel", "true", {"true", "false"}},
          {"label_color", "Label Color", "255;236;120", {},
           common::DisplayPropertyKind::kColor},
          {"label_height", "Label Height (m)", "0.22"}};
}

void RadarScanDisplay::onEnable() {
  ChannelDisplay::onEnable();
  ensureHud();
  hud_dirty_ = true;
}

void RadarScanDisplay::onDisable() {
  if (hud_) {
    hud_->hide();
  }
  ChannelDisplay::onDisable();
}

void RadarScanDisplay::onPropertyChanged(const std::string& /*key*/) {
  hud_dirty_ = true;
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void RadarScanDisplay::clearReceivedData() {
  points_.clear();
  primary_range_ = -1.f;
  primary_angle_deg_ = 0.f;
  primary_velocity_mps_ = 0.f;
  return_count_ = 0;
  have_angle_vel_ = false;
  hud_dirty_ = true;
}

void RadarScanDisplay::ensureHud() {
  if (hud_) {
    return;
  }
  hud_ = std::make_shared<viz::OverlayObject>("RadarScanHud/" + name());
}

void RadarScanDisplay::DrawHalo(rendering::SceneOverlay& scene,
                                const QVector3D& center, float radius,
                                const QColor& color) {
  constexpr int kSegs = 28;
  std::vector<QVector3D> ring;
  ring.reserve(kSegs + 1);
  for (int i = 0; i <= kSegs; ++i) {
    const float t = 2.f * kPi * static_cast<float>(i) / static_cast<float>(kSegs);
    ring.emplace_back(center.x() + radius * std::cos(t),
                      center.y() + radius * std::sin(t), center.z());
  }
  scene.addLineLoop(ring, color, 1.6f);
}

void RadarScanDisplay::DrawBillboardLabel(rendering::SceneOverlay& scene,
                                          const QVector3D& pos,
                                          const QString& text,
                                          const QColor& color,
                                          float char_height_m) {
  if (text.isEmpty()) {
    return;
  }
  const float h = std::max(0.10f, char_height_m);
  const int pixel_h = static_cast<int>(std::clamp(h * 180.f, 28.f, 160.f));
  const QImage img = rendering::RasterizeTextLabel(text, color, pixel_h);
  if (img.isNull()) {
    return;
  }
  const float aspect = static_cast<float>(img.width()) /
                       static_cast<float>(std::max(1, img.height()));
  const float half_h = h * 0.5f;
  scene.addViewFacingTexturedQuad(pos, half_h * aspect, half_h, img);
}

void RadarScanDisplay::refreshHud() {
  if (propertyValue("show_hud", "true") != "true") {
    if (hud_) {
      hud_->hide();
    }
    return;
  }
  ensureHud();
  if (!hud_) {
    return;
  }

  constexpr int kW = 300;
  constexpr int kH = 188;
  hud_->updateTextureSize(kW, kH);
  hud_->setDimensions(kW, kH);
  hud_->setPosition(16, 16);

  auto buffer = hud_->getBuffer();
  QColor bg(0, 0, 0, 0);
  QImage image = buffer.getQImage(kW, kH, bg);
  if (image.isNull()) {
    return;
  }

  QPainter p(&image);
  p.setRenderHint(QPainter::Antialiasing, true);
  QPainterPath card;
  card.addRoundedRect(QRectF(1.5, 1.5, kW - 3.0, kH - 3.0), 14, 14);
  p.fillPath(card, QColor(10, 16, 26, 218));
  p.setPen(QPen(QColor(70, 190, 255, 180), 1.4));
  p.drawPath(card);

  p.setPen(QColor(90, 210, 255));
  p.setFont(QFont("Sans Serif", 11, QFont::Bold));
  p.drawText(18, 26, QString::fromUtf8("ICL1122  ·  目标信息"));

  p.setPen(QColor(70, 90, 110));
  p.drawLine(18, 34, kW - 18, 34);

  char rb[48];
  char ab[48];
  char vb[48];
  if (primary_range_ >= 0.f) {
    std::snprintf(rb, sizeof(rb), "%.2f m", primary_range_);
  } else {
    std::snprintf(rb, sizeof(rb), "--");
  }
  if (have_angle_vel_) {
    std::snprintf(ab, sizeof(ab), "%+.1f°", primary_angle_deg_);
    std::snprintf(vb, sizeof(vb), "%+.2f m/s", primary_velocity_mps_);
  } else {
    std::snprintf(ab, sizeof(ab), "--");
    std::snprintf(vb, sizeof(vb), "--");
  }

  auto metric = [&](int y, const QString& title, const QColor& value_color,
                    const char* value) {
    p.setPen(QColor(150, 165, 180));
    p.setFont(QFont("Sans Serif", 9, QFont::DemiBold));
    p.drawText(20, y, title);
    p.setPen(value_color);
    p.setFont(QFont("Sans Serif", 18, QFont::Bold));
    p.drawText(108, y + 2, QString::fromUtf8(value));
  };
  metric(62, QString::fromUtf8("最近距离"), QColor(90, 255, 160), rb);
  metric(102, QString::fromUtf8("方位角"), QColor(255, 214, 90), ab);
  metric(142, QString::fromUtf8("径向速度"), QColor(110, 200, 255), vb);

  p.setPen(QColor(120, 140, 155));
  p.setFont(QFont("Sans Serif", 9));
  p.drawText(20, 174,
             QString::fromUtf8("回波 %1  ·  H FOV ±43°").arg(return_count_));
  p.end();

  hud_->show();
  hud_dirty_ = false;
}

void RadarScanDisplay::processMessage(
    const automsgs::msgs::sensor_msgs::RadarScan& message) {
  points_.clear();
  primary_range_ = -1.f;
  primary_angle_deg_ = 0.f;
  primary_velocity_mps_ = 0.f;
  return_count_ = 0;
  have_angle_vel_ = false;
  if (context_ == nullptr) {
    return;
  }

  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();
  automsgs::msgs::geometry_msgs::TransformStamped tf;
  bool have_tf = false;
  if (frame != context_->fixed_frame) {
    try {
      tf = context_->tf_buffer->lookupTransform(
          context_->fixed_frame, frame, message.header().stamp());
      have_tf = true;
    } catch (...) {
      setStatusWarn("TF missing: " + frame + " -> " + context_->fixed_frame);
      return;
    }
  }

  const QColor flat_color = common::ParseColorProperty(
      propertyValue("color", "255;255;255"), QColor(255, 255, 255));
  const std::string color_mode = propertyValue("color_transform", "Range");

  float min_a = std::numeric_limits<float>::max();
  float max_a = std::numeric_limits<float>::lowest();
  float min_d = std::numeric_limits<float>::max();
  float max_d = std::numeric_limits<float>::lowest();
  float min_r = std::numeric_limits<float>::max();
  float max_r = std::numeric_limits<float>::lowest();
  for (int i = 0; i < message.returns_size(); ++i) {
    const auto& r = message.returns(i);
    min_a = std::min(min_a, r.amplitude());
    max_a = std::max(max_a, r.amplitude());
    min_d = std::min(min_d, r.doppler_velocity());
    max_d = std::max(max_d, r.doppler_velocity());
    if (std::isfinite(r.range()) && r.range() >= 0.f) {
      min_r = std::min(min_r, r.range());
      max_r = std::max(max_r, r.range());
    }
  }
  if (!(min_a < max_a)) {
    min_a = 0.f;
    max_a = 1.f;
  }
  if (!(min_d < max_d)) {
    min_d = -1.f;
    max_d = 1.f;
  }
  if (!(min_r < max_r)) {
    min_r = 0.f;
    max_r = 6.f;
  }

  int nearest_index = -1;
  points_.reserve(static_cast<std::size_t>(message.returns_size()));
  for (int i = 0; i < message.returns_size(); ++i) {
    const auto& r = message.returns(i);
    if (!std::isfinite(r.range()) || r.range() < 0.f) {
      continue;
    }
    const float ce = std::cos(r.elevation());
    QVector3D local(r.range() * ce * std::sin(r.azimuth()),
                    r.range() * ce * std::cos(r.azimuth()),
                    r.range() * std::sin(r.elevation()));
    if (have_tf) {
      local = transformPoint(tf, local);
    }
    QColor color = flat_color;
    if (color_mode == "Range") {
      color = MixRangeColor(r.range(), std::max(max_r, 1.f));
    } else if (color_mode == "Amplitude") {
      color = colorFromIntensity(r.amplitude(), min_a, max_a);
    } else if (color_mode == "Doppler") {
      color = colorFromScalar(r.doppler_velocity(), min_d, max_d);
    }
    const float az_deg = r.azimuth() * kRad2Deg;
    points_.push_back(
        {local, color, r.range(), az_deg, r.doppler_velocity(), r.amplitude(),
         false});
    if (nearest_index < 0 || r.range() < points_[static_cast<std::size_t>(nearest_index)].range) {
      nearest_index = static_cast<int>(points_.size()) - 1;
      primary_range_ = r.range();
      primary_angle_deg_ = az_deg;
      primary_velocity_mps_ = r.doppler_velocity();
    }
    if (std::fabs(r.azimuth()) > 1e-4f ||
        std::fabs(r.doppler_velocity()) > 1e-4f) {
      have_angle_vel_ = true;
    }
  }
  if (nearest_index >= 0) {
    points_[static_cast<std::size_t>(nearest_index)].primary = true;
    points_[static_cast<std::size_t>(nearest_index)].color = QColor(90, 255, 160);
  }
  return_count_ = static_cast<int>(points_.size());
  hud_dirty_ = true;

  setStatusOk();
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void RadarScanDisplay::onDraw(rendering::SceneOverlay& scene) {
  refreshHud();

  const float alpha = std::clamp(
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f), 0.f, 1.f);
  const bool show_spokes = propertyValue("show_spokes", "true") == "true";
  const bool show_halo = propertyValue("show_halo", "true") == "true";
  const QVector3D origin(0.f, 0.f, 0.05f);

  for (const ScanPoint& point : points_) {
    if (show_spokes) {
      scene.addLine(origin, point.position,
                    WithAlpha(point.color, alpha * (point.primary ? 0.55f : 0.22f)));
    }
    const QVector3D ground(point.position.x(), point.position.y(), 0.04f);
    scene.addLine(ground, point.position, WithAlpha(point.color, alpha * 0.35f));
    if (show_halo) {
      DrawHalo(scene, QVector3D(point.position.x(), point.position.y(), 0.06f),
               point.primary ? 0.18f : 0.12f,
               WithAlpha(point.color, alpha * (point.primary ? 0.9f : 0.55f)));
    }
  }

  const float point_size =
      common::ParseFloatProperty(propertyValue("point_size", "28"), 28.f);
  const rendering::PointCloudStyle style =
      rendering::parsePointCloudStyle(propertyValue("style", "Spheres"));
  std::vector<ColoredPoint3D> colored;
  colored.reserve(points_.size());
  for (const auto& point : points_) {
    colored.push_back({point.position, WithAlpha(point.color, alpha)});
  }
  drawColoredPointsOgreOrGl(context_, scene, name(), typeId(), point_size, style,
                            colored, true);

  if (propertyValue("show_range", "true") != "true") {
    return;
  }
  const QColor label_color = common::ParseColorProperty(
      propertyValue("label_color", "255;236;120"), QColor(255, 236, 120));
  const float label_height =
      common::ParseFloatProperty(propertyValue("label_height", "0.22"), 0.22f);
  for (const ScanPoint& point : points_) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.2fm", point.range);
    const QColor c = point.primary ? QColor(90, 255, 160) : label_color;
    DrawBillboardLabel(scene,
                       point.position + QVector3D(0.28f, 0.02f, 0.16f),
                       QString::fromUtf8(buf), c,
                       point.primary ? label_height * 1.15f : label_height);
  }
}

}  // namespace display
}  // namespace autoviz

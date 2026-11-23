/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/radar/radar_fov_display.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <QColor>
#include <QFont>
#include <QFontMetrics>
#include <QImage>
#include <QPainter>
#include <QString>
#include <QVector3D>

#include "autoviz/common/display_property.hpp"

namespace autoviz {
namespace display {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDeg2Rad = kPi / 180.f;
constexpr float kZ = 0.04f;

QColor WithAlpha(QColor c, float alpha) {
  c.setAlphaF(std::clamp(alpha, 0.f, 1.f));
  return c;
}

QVector3D Polar(float range, float azimuth, float z) {
  return {range * std::sin(azimuth), range * std::cos(azimuth), z};
}

void FillQuad(rendering::SceneOverlay& scene, const QVector3D& a,
              const QVector3D& b, const QVector3D& c, const QVector3D& d,
              const QColor& ca, const QColor& cb, const QColor& cc,
              const QColor& cd) {
  scene.addColoredTriangle(a, b, c, ca, cb, cc);
  scene.addColoredTriangle(a, c, d, ca, cc, cd);
}

void FillQuad(rendering::SceneOverlay& scene, const QVector3D& a,
              const QVector3D& b, const QVector3D& c, const QVector3D& d,
              const QColor& color) {
  FillQuad(scene, a, b, c, d, color, color, color, color);
}

void FillSectorBand(rendering::SceneOverlay& scene, float r0, float r1,
                    float az0, float az1, int segs, float z,
                    const QColor& inner, const QColor& outer) {
  const int n = std::max(8, segs);
  for (int i = 0; i < n; ++i) {
    const float t0 =
        az0 + (az1 - az0) * (static_cast<float>(i) / static_cast<float>(n));
    const float t1 =
        az0 + (az1 - az0) * (static_cast<float>(i + 1) / static_cast<float>(n));
    FillQuad(scene, Polar(r0, t0, z), Polar(r0, t1, z), Polar(r1, t1, z),
             Polar(r1, t0, z), inner, inner, outer, outer);
  }
}

void DrawPolyline(rendering::SceneOverlay& scene,
                  const std::vector<QVector3D>& pts, const QColor& color) {
  for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
    scene.addLine(pts[i], pts[i + 1], color);
  }
}

std::vector<QVector3D> ArcPoints(float range, float az0, float az1, int segs,
                                 float z) {
  const int n = std::max(8, segs);
  std::vector<QVector3D> pts;
  pts.reserve(static_cast<std::size_t>(n + 1));
  for (int i = 0; i <= n; ++i) {
    const float t =
        az0 + (az1 - az0) * (static_cast<float>(i) / static_cast<float>(n));
    pts.push_back(Polar(range, t, z));
  }
  return pts;
}

QImage MakeOutlinedLabel(const QString& text, const QColor& color,
                         int pixel_h) {
  if (text.isEmpty()) {
    return {};
  }
  constexpr int kSs = 2;
  const int font_px = std::max(12, pixel_h * kSs);
  QFont font;
  font.setPixelSize(font_px);
  font.setWeight(QFont::DemiBold);
  const QFontMetrics metrics(font);
  const QRect bounds = metrics.boundingRect(text);
  const int pad = 4 * kSs;
  QImage image(bounds.width() + pad * 2, bounds.height() + pad * 2,
               QImage::Format_RGBA8888);
  image.fill(Qt::transparent);
  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::TextAntialiasing, true);
  painter.setFont(font);
  const int x = pad;
  const int y = pad + metrics.ascent();
  painter.setPen(QColor(10, 12, 16, 230));
  for (int dx = -2; dx <= 2; ++dx) {
    for (int dy = -2; dy <= 2; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      painter.drawText(x + dx, y + dy, text);
    }
  }
  painter.setPen(color);
  painter.drawText(x, y, text);
  return image;
}

void DrawBillboard(rendering::SceneOverlay& scene, const QVector3D& pos,
                   const QString& text, const QColor& color, float char_h) {
  const float h = std::max(0.14f, char_h);
  const int pixel_h = static_cast<int>(std::clamp(h * 220.f, 36.f, 180.f));
  const QImage img = MakeOutlinedLabel(text, color, pixel_h);
  if (img.isNull()) {
    return;
  }
  const float aspect = static_cast<float>(img.width()) /
                       static_cast<float>(std::max(1, img.height()));
  const float half_h = h * 0.5f;
  scene.addViewFacingTexturedQuad(pos, half_h * aspect, half_h, img);
}

}  // namespace

RadarFovDisplay::RadarFovDisplay() { setProperties({}); }

std::vector<common::DisplayPropertySpec> RadarFovDisplay::propertySpecs()
    const {
  return {{"max_range", "Max Range (m)", "6"},
          {"half_fov_deg", "Half FOV (deg)", "43"},
          {"half_el_deg", "Half Elevation (deg)", "14"},
          {"road_half_width", "Road Half Width (m)", "0.8"},
          {"ring_interval", "Ring Interval (m)", "1"},
          {"show_road", "Show Road", "false", {"true", "false"}},
          {"show_fov", "Show FOV", "true", {"true", "false"}},
          {"show_beam", "Show 3D Beam", "false", {"true", "false"}},
          {"show_rings", "Show Rings", "true", {"true", "false"}},
          {"show_labels", "Show Labels", "true", {"true", "false"}},
          {"road_color", "Road Color", "36;40;48", {},
           common::DisplayPropertyKind::kColor},
          {"border_color", "Border Color", "28;72;48", {},
           common::DisplayPropertyKind::kColor},
          {"fov_color", "FOV Color", "48;196;255", {},
           common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "0.14"},
          {"ring_color", "Ring Color", "210;228;240", {},
           common::DisplayPropertyKind::kColor},
          {"ring_alpha", "Ring Alpha", "0.80"},
          {"edge_alpha", "Edge Alpha", "0.90"},
          {"label_color", "Label Color", "240;245;250", {},
           common::DisplayPropertyKind::kColor},
          {"road_alpha", "Road Alpha", "0.35"}};
}

void RadarFovDisplay::onPropertyChanged(const std::string& /*key*/) {
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void RadarFovDisplay::onDraw(rendering::SceneOverlay& scene) {
  const float max_range = std::max(
      1.f, common::ParseFloatProperty(propertyValue("max_range", "6"), 6.f));
  const float half_fov_deg = std::clamp(
      common::ParseFloatProperty(propertyValue("half_fov_deg", "43"), 43.f),
      1.f, 89.f);
  const float half_el_deg = std::clamp(
      common::ParseFloatProperty(propertyValue("half_el_deg", "14"), 14.f), 0.f,
      80.f);
  const float half_fov = half_fov_deg * kDeg2Rad;
  const float half_el = half_el_deg * kDeg2Rad;
  const float road_hw = std::max(
      0.2f,
      common::ParseFloatProperty(propertyValue("road_half_width", "0.8"), 0.8f));
  const float ring_interval = std::max(
      0.25f,
      common::ParseFloatProperty(propertyValue("ring_interval", "1"), 1.f));
  const bool show_road = propertyValue("show_road", "false") == "true";
  const bool show_fov = propertyValue("show_fov", "true") == "true";
  const bool show_beam = propertyValue("show_beam", "false") == "true";
  const bool show_rings = propertyValue("show_rings", "true") == "true";
  const bool show_labels = propertyValue("show_labels", "true") == "true";
  const float fov_alpha = std::clamp(
      common::ParseFloatProperty(
          propertyValue("alpha", propertyValue("fov_alpha", "0.14")), 0.14f),
      0.f, 1.f);
  const float ring_alpha = std::clamp(
      common::ParseFloatProperty(propertyValue("ring_alpha", "0.80"), 0.80f),
      0.f, 1.f);
  const float edge_alpha = std::clamp(
      common::ParseFloatProperty(propertyValue("edge_alpha", "0.90"), 0.90f),
      0.f, 1.f);
  const float road_alpha = std::clamp(
      common::ParseFloatProperty(propertyValue("road_alpha", "0.35"), 0.35f),
      0.f, 1.f);

  const QColor road_color = WithAlpha(
      common::ParseColorProperty(propertyValue("road_color", "36;40;48"),
                                 QColor(36, 40, 48)),
      road_alpha);
  const QColor border_color = WithAlpha(
      common::ParseColorProperty(propertyValue("border_color", "28;72;48"),
                                 QColor(28, 72, 48)),
      road_alpha);
  const QColor fov_base = common::ParseColorProperty(
      propertyValue("fov_color", "48;196;255"), QColor(48, 196, 255));
  const QColor ring_color = WithAlpha(
      common::ParseColorProperty(propertyValue("ring_color", "210;228;240"),
                                 QColor(210, 228, 240)),
      ring_alpha);
  const QColor label_color =
      common::ParseColorProperty(propertyValue("label_color", "240;245;250"),
                                 QColor(240, 245, 250));

  const QVector3D origin(0.f, 0.f, kZ);
  const int arc_segs = 56;
  const float label_h = std::clamp(max_range * 0.055f, 0.18f, 0.38f);

  if (show_road) {
    const float border_w = road_hw * 0.45f;
    FillQuad(scene, QVector3D(-(road_hw + border_w), 0.f, kZ),
             QVector3D(-road_hw, 0.f, kZ), QVector3D(-road_hw, max_range, kZ),
             QVector3D(-(road_hw + border_w), max_range, kZ), border_color);
    FillQuad(scene, QVector3D(road_hw, 0.f, kZ),
             QVector3D(road_hw + border_w, 0.f, kZ),
             QVector3D(road_hw + border_w, max_range, kZ),
             QVector3D(road_hw, max_range, kZ), border_color);
    FillQuad(scene, QVector3D(-road_hw, 0.f, kZ), QVector3D(road_hw, 0.f, kZ),
             QVector3D(road_hw, max_range, kZ),
             QVector3D(-road_hw, max_range, kZ), road_color);
    const QColor edge(210, 220, 230, 160);
    scene.addLine(QVector3D(-road_hw, 0.f, kZ + 0.02f),
                  QVector3D(-road_hw, max_range, kZ + 0.02f), edge);
    scene.addLine(QVector3D(road_hw, 0.f, kZ + 0.02f),
                  QVector3D(road_hw, max_range, kZ + 0.02f), edge);
  }

  if (show_fov) {
    FillSectorBand(scene, 0.f, max_range, -half_fov, half_fov, arc_segs, kZ,
                   WithAlpha(fov_base, fov_alpha * 0.25f),
                   WithAlpha(fov_base, fov_alpha));

    const QColor edge = WithAlpha(fov_base, edge_alpha);
    scene.addLine(origin, Polar(max_range, -half_fov, kZ + 0.01f), edge);
    scene.addLine(origin, Polar(max_range, half_fov, kZ + 0.01f), edge);
    DrawPolyline(scene,
                 ArcPoints(max_range, -half_fov, half_fov, arc_segs, kZ + 0.012f),
                 edge);

    const QColor boresight(255, 196, 72,
                           static_cast<int>(std::clamp(edge_alpha, 0.f, 1.f) * 255.f));
    scene.addLine(origin, QVector3D(0.f, max_range, kZ + 0.012f), boresight);
  }

  if (show_beam && half_el > 1e-4f) {
    const float ce = std::cos(half_el);
    const float se = std::sin(half_el);
    auto corner = [&](float az, float el_sign) {
      return QVector3D(max_range * ce * std::sin(az),
                       max_range * ce * std::cos(az),
                       max_range * se * el_sign);
    };
    const QVector3D o(0.f, 0.f, 0.f);
    const QVector3D lu = corner(-half_fov, 1.f);
    const QVector3D ld = corner(-half_fov, -1.f);
    const QVector3D ru = corner(half_fov, 1.f);
    const QVector3D rd = corner(half_fov, -1.f);
    const QColor rim = WithAlpha(fov_base, edge_alpha * 0.5f);
    scene.addLine(o, lu, rim);
    scene.addLine(o, ru, rim);
    scene.addLine(o, ld, rim);
    scene.addLine(o, rd, rim);
    scene.addLine(lu, ru, rim);
    scene.addLine(ld, rd, rim);
    scene.addLine(lu, ld, rim);
    scene.addLine(ru, rd, rim);
  }

  // Origin chevron — radar looking +Y.
  {
    const QColor body(255, 255, 255, 230);
    const QColor glow = WithAlpha(fov_base, 0.55f);
    const QVector3D tip(0.f, std::min(0.28f, max_range * 0.06f), kZ + 0.05f);
    const float w = std::min(0.14f, max_range * 0.03f);
    scene.addColoredTriangle(tip, QVector3D(-w, -w * 0.55f, kZ + 0.05f),
                             QVector3D(w, -w * 0.55f, kZ + 0.05f), body, body,
                             body);
    scene.addLineLoop({QVector3D(-w * 1.35f, -w * 0.7f, kZ + 0.04f),
                       QVector3D(0.f, tip.y() * 1.15f, kZ + 0.04f),
                       QVector3D(w * 1.35f, -w * 0.7f, kZ + 0.04f)},
                      glow, 1.4f);
  }

  if (show_rings || show_labels) {
    const QColor minor = WithAlpha(ring_color, ring_alpha * 0.55f);
    const QColor major = WithAlpha(ring_color, std::min(1.f, ring_alpha));
    const QColor radial = WithAlpha(ring_color, ring_alpha * 0.35f);
    int ring_index = 0;
    for (float r = ring_interval; r <= max_range + 0.01f; r += ring_interval) {
      ++ring_index;
      const bool is_major =
          (ring_index % 2 == 0) || std::fabs(r - max_range) < 0.05f;
      if (show_rings) {
        DrawPolyline(scene,
                     ArcPoints(r, -half_fov, half_fov, arc_segs, kZ + 0.02f),
                     is_major ? major : minor);
        if (is_major) {
          for (float deg : {-30.f, -15.f, 15.f, 30.f}) {
            if (std::fabs(deg) >= half_fov_deg - 0.5f) {
              continue;
            }
            scene.addLine(origin, Polar(r, deg * kDeg2Rad, kZ + 0.015f), radial);
          }
        }
      }
      if (show_labels) {
        // Sit just left of the 0° ray so numbers don't collide with ±FOV.
        const QVector3D pos =
            Polar(r, 0.f, 0.24f) +
            QVector3D(-std::max(0.32f, max_range * 0.05f), 0.f, 0.f);
        DrawBillboard(scene, pos,
                      QString::number(static_cast<int>(std::lround(r))) +
                          QStringLiteral("m"),
                      label_color, label_h);
      }
    }
  }

  if (show_labels) {
    DrawBillboard(scene, QVector3D(0.f, -std::max(0.32f, max_range * 0.055f), 0.28f),
                  QStringLiteral("RADAR"), QColor(90, 255, 190), label_h * 0.95f);
    DrawBillboard(scene, Polar(max_range * 1.12f, 0.f, 0.24f),
                  QStringLiteral("0°"), QColor(255, 210, 90), label_h * 0.85f);
    DrawBillboard(scene, Polar(max_range * 1.10f, -half_fov, 0.24f),
                  QString("%1°").arg(static_cast<int>(std::lround(-half_fov_deg))),
                  QColor(140, 220, 255), label_h * 0.8f);
    DrawBillboard(scene, Polar(max_range * 1.10f, half_fov, 0.24f),
                  QString("%1°").arg(static_cast<int>(std::lround(half_fov_deg))),
                  QColor(140, 220, 255), label_h * 0.8f);
  }

  setStatusOk();
}

}  // namespace display
}  // namespace autoviz

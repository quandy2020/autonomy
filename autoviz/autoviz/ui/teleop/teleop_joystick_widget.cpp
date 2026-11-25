/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/teleop/teleop_joystick_widget.hpp"

#include <algorithm>
#include <cmath>

#include <QEvent>
#include <QFontMetrics>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QRadialGradient>
#include <QResizeEvent>

#include "autoviz/ui/theme/tokens.hpp"

namespace autoviz {
namespace teleop {
namespace {

constexpr double kWellRatio = 0.62;   // thumbstick bowl / outer
constexpr double kKnobRatio = 0.22;   // knob head / outer
constexpr double kTravelRatio = 0.28; // max knob offset / outer
constexpr int kLabelBand = 0;
constexpr int kMinPadSide = 108;

/** Flat neutrals — accent comes from theme::Shell() at paint time. */
constexpr QColor kIcon(255, 255, 255);
constexpr QColor kWell(0xF1, 0xF5, 0xF9);
constexpr QColor kWellRim(0xCB, 0xD5, 0xE1);
constexpr QColor kKnob(0xFF, 0xFF, 0xFF);
constexpr QColor kKnobRim(0x94, 0xA3, 0xB8);

QColor WithAlpha(QColor color, int alpha) {
  color.setAlpha(alpha);
  return color;
}

QColor ThemeAccent() { return theme::Shell().accent; }

constexpr double kPressThreshold = 0.28;

QPointF Polar(const QPointF& c, double angle_rad, double r) {
  return QPointF(c.x() + std::cos(angle_rad) * r, c.y() + std::sin(angle_rad) * r);
}

void DrawDirSector(QPainter& painter, const QPointF& c, double outer_r,
                   double inner_r, double angle_rad, double half_span_rad,
                   const QColor& fill) {
  // Flat pie wedge between inner and outer radius around direction angle.
  constexpr int kSteps = 12;
  QPainterPath path;
  path.moveTo(Polar(c, angle_rad - half_span_rad, inner_r));
  for (int i = 0; i <= kSteps; ++i) {
    const double t = static_cast<double>(i) / kSteps;
    const double a = angle_rad - half_span_rad + t * (2.0 * half_span_rad);
    path.lineTo(Polar(c, a, outer_r));
  }
  for (int i = kSteps; i >= 0; --i) {
    const double t = static_cast<double>(i) / kSteps;
    const double a = angle_rad - half_span_rad + t * (2.0 * half_span_rad);
    path.lineTo(Polar(c, a, inner_r));
  }
  path.closeSubpath();
  painter.fillPath(path, fill);
}

void DrawTriangle(QPainter& painter, const QPointF& tip, const QPointF& left,
                  const QPointF& right, const QColor& fill) {
  QPainterPath path;
  path.moveTo(tip);
  path.lineTo(left);
  path.lineTo(right);
  path.closeSubpath();
  painter.fillPath(path, fill);
}

void DrawDirTriangle(QPainter& painter, const QPointF& c, double angle_rad,
                     double mid_r, double size, const QColor& fill) {
  const QPointF tip = Polar(c, angle_rad, mid_r + size * 0.55);
  const QPointF base = Polar(c, angle_rad, mid_r - size * 0.45);
  const QPointF n(-std::sin(angle_rad), std::cos(angle_rad));
  DrawTriangle(painter, tip, base + n * (size * 0.55), base - n * (size * 0.55),
               fill);
}

void DrawDirDouble(QPainter& painter, const QPointF& c, double angle_rad,
                   double mid_r, double size, const QColor& fill) {
  DrawDirTriangle(painter, c, angle_rad, mid_r - size * 0.28, size * 0.85, fill);
  DrawDirTriangle(painter, c, angle_rad, mid_r + size * 0.32, size * 0.85, fill);
}

}  // namespace

TeleopJoystickWidget::TeleopJoystickWidget(const QString& label, QWidget* parent)
    : label_(label), QWidget(parent) {
  setMinimumSize(112, 112);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setMouseTracking(true);
  setCursor(Qt::OpenHandCursor);
  setFocusPolicy(Qt::NoFocus);
  setAttribute(Qt::WA_StyledBackground, true);
}

QSize TeleopJoystickWidget::sizeHint() const { return QSize(160, 160); }

QSize TeleopJoystickWidget::minimumSizeHint() const { return QSize(112, 112); }

QPointF TeleopJoystickWidget::normalizedValue() const { return knob_; }

void TeleopJoystickWidget::setLabel(const QString& label) {
  if (label_ == label) {
    return;
  }
  label_ = label;
  update();
}

void TeleopJoystickWidget::setAxes(TeleopStickAxes axes) {
  if (axes_ == axes) {
    return;
  }
  axes_ = axes;
  knob_ = clampToAxes(knob_);
  update();
}

void TeleopJoystickWidget::setRoyalStyle(bool enabled) {
  if (royal_style_ == enabled) {
    return;
  }
  royal_style_ = enabled;
  // Never set WA_TranslucentBackground here: on X11 it punches a hole through
  // the main Autoviz window when the pad is still parented to the docked panel.
  setAttribute(Qt::WA_TranslucentBackground, false);
  setAttribute(Qt::WA_NoSystemBackground, enabled);
  setAttribute(Qt::WA_StyledBackground, !enabled);
  setAutoFillBackground(false);
  update();
}

void TeleopJoystickWidget::setNormalizedValue(const QPointF& value,
                                              bool emit_signal) {
  const QPointF clamped = clampToAxes(value);
  if (std::hypot(clamped.x() - knob_.x(), clamped.y() - knob_.y()) < 1e-4) {
    return;
  }
  knob_ = clamped;
  update();
  if (emit_signal) {
    emit valueChanged(knob_.x(), knob_.y());
  }
}

void TeleopJoystickWidget::reset() {
  resetVisual();
  emit valueChanged(0.0, 0.0);
  emit released();
}

void TeleopJoystickWidget::resetVisual() {
  dragging_ = false;
  setCursor(Qt::OpenHandCursor);
  if (knob_.isNull() && !isVisible()) {
    return;
  }
  knob_ = QPointF(0.0, 0.0);
  if (mouseGrabber() == this) {
    releaseMouse();
  }
  update();
}

QRectF TeleopJoystickWidget::outerRect() const {
  const int inset = royal_style_ ? 2 : 8;
  const int side = std::max(
      kMinPadSide, std::min(width(), height() - kLabelBand) - inset);
  const double x = (width() - side) * 0.5;
  const double y = (height() - kLabelBand - side) * 0.5;
  return QRectF(x, y, side, side);
}

QPointF TeleopJoystickWidget::center() const { return outerRect().center(); }

double TeleopJoystickWidget::radius() const { return outerRect().width() * 0.5; }

QPointF TeleopJoystickWidget::clampToAxes(const QPointF& vector) const {
  QPointF v = vector;
  if (axes_ == TeleopStickAxes::kYaw) {
    v.setY(0.0);
  }
  const double length = std::hypot(v.x(), v.y());
  if (length <= 1.0 || length <= 1e-6) {
    return v;
  }
  return v / length;
}

void TeleopJoystickWidget::updateFromPosition(const QPointF& local_pos) {
  // Thumbstick: drag relative to bowl center; travel stays inside the well.
  const QPointF center_pt = center();
  const double travel_r = radius() * kTravelRatio;
  QPointF delta(local_pos.x() - center_pt.x(), local_pos.y() - center_pt.y());
  if (travel_r > 1e-3) {
    delta /= travel_r;
  }
  const QPointF clamped = clampToAxes(delta);
  if (std::hypot(clamped.x() - knob_.x(), clamped.y() - knob_.y()) < 1e-4) {
    return;
  }
  knob_ = clamped;
  update();
  emit valueChanged(knob_.x(), knob_.y());
}

void TeleopJoystickWidget::paintEvent(QPaintEvent* /*event*/) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);

  const QRectF outer = outerRect();
  if (outer.width() < 8.0) {
    return;
  }
  const QPointF c = outer.center();
  const double R = outer.width() * 0.5;
  const double well_r = R * kWellRatio;
  const double knob_r = R * kKnobRatio;
  const double travel = R * kTravelRatio;
  if (royal_style_) {
    painter.setCompositionMode(QPainter::CompositionMode_Source);
    painter.fillRect(rect(), Qt::transparent);
    painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
    paintRoyal(painter, c, R, well_r, knob_r, travel);
  } else {
    paintDocked(painter, c, R, well_r, knob_r, travel);
  }
}

void TeleopJoystickWidget::paintDocked(QPainter& painter, const QPointF& c,
                                       double R, double well_r, double knob_r,
                                       double travel) {
  const bool active = dragging_ || std::hypot(knob_.x(), knob_.y()) > 1e-3;
  const bool omni = axes_ == TeleopStickAxes::kOmni;
  constexpr double kPi = 3.14159265358979323846;
  const QPointF stick(c.x() + knob_.x() * travel, c.y() + knob_.y() * travel);

  const QColor accent = ThemeAccent();
  const QColor ring = active ? accent.darker(118) : accent;
  const QColor sector = accent.darker(140);
  const QColor icon_press = accent.lighter(160);

  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(15, 23, 42, 26));
  painter.drawEllipse(QRectF(c.x() - R, c.y() - R, R * 2.0, R * 2.0)
                          .translated(0, 3));

  {
    QPainterPath ring_path;
    ring_path.addEllipse(c, R, R);
    QPainterPath hole;
    hole.addEllipse(c, well_r, well_r);
    ring_path = ring_path.subtracted(hole);
    painter.fillPath(ring_path, ring);
  }

  const bool press_f = omni && knob_.y() < -kPressThreshold;
  const bool press_b = omni && knob_.y() > kPressThreshold;
  const bool press_l = knob_.x() < -kPressThreshold;
  const bool press_r = knob_.x() > kPressThreshold;
  constexpr double kHalfSpan = 0.42;
  painter.setPen(Qt::NoPen);
  if (press_f) {
    DrawDirSector(painter, c, R, well_r, -kPi * 0.5, kHalfSpan, sector);
  }
  if (press_b) {
    DrawDirSector(painter, c, R, well_r, kPi * 0.5, kHalfSpan, sector);
  }
  if (press_l) {
    DrawDirSector(painter, c, R, well_r, kPi, kHalfSpan, sector);
  }
  if (press_r) {
    DrawDirSector(painter, c, R, well_r, 0.0, kHalfSpan, sector);
  }

  const double glyph_r = (well_r + R) * 0.5;
  const double glyph = std::max(6.5, R * 0.10);
  const auto glyph_color = [&](bool pressed) {
    return pressed ? icon_press : kIcon;
  };
  const auto glyph_size = [&](bool pressed) {
    return pressed ? glyph * 1.18 : glyph;
  };
  painter.setPen(Qt::NoPen);
  if (omni) {
    DrawDirTriangle(painter, c, -kPi * 0.5, glyph_r, glyph_size(press_f),
                    glyph_color(press_f));
    DrawDirTriangle(painter, c, kPi * 0.5, glyph_r, glyph_size(press_b),
                    glyph_color(press_b));
  }
  DrawDirDouble(painter, c, kPi, glyph_r, glyph_size(press_l) * 0.92,
                glyph_color(press_l));
  DrawDirDouble(painter, c, 0.0, glyph_r, glyph_size(press_r) * 0.92,
                glyph_color(press_r));

  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(15, 23, 42, 20));
  painter.drawEllipse(c + QPointF(0, 1.5), well_r, well_r);
  painter.setBrush(kWell);
  painter.setPen(QPen(kWellRim, 1.2));
  painter.drawEllipse(c, well_r, well_r);
  painter.setPen(QPen(QColor(148, 163, 184, 90), 1.0));
  painter.setBrush(Qt::NoBrush);
  painter.drawEllipse(c, well_r - 3.0, well_r - 3.0);

  {
    const QPointF shadow =
        stick + QPointF(-knob_.x() * 1.5, 2.0 + std::abs(knob_.y()) * 1.2);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(15, 23, 42, active ? 40 : 28));
    painter.drawEllipse(shadow, knob_r * 1.05, knob_r * 0.92);
  }

  painter.setBrush(kKnob);
  painter.setPen(QPen(active ? ring : kKnobRim, 1.6));
  painter.drawEllipse(stick, knob_r, knob_r);
  painter.setPen(Qt::NoPen);
  painter.setBrush(ring);
  painter.drawEllipse(stick, knob_r * 0.34, knob_r * 0.34);
}

void TeleopJoystickWidget::paintRoyal(QPainter& painter, const QPointF& c,
                                      double R, double well_r, double knob_r,
                                      double travel) {
  const bool omni = axes_ == TeleopStickAxes::kOmni;
  constexpr double kPi = 3.14159265358979323846;
  const QPointF stick(c.x() + knob_.x() * travel, c.y() + knob_.y() * travel);
  const QColor accent = WithAlpha(ThemeAccent(), 220);
  const QColor well = WithAlpha(ThemeAccent(), 42);
  QColor knob = ThemeAccent().lighter(165);
  knob.setAlpha(235);

  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(12, 10, 8, 150));
  painter.drawEllipse(c, R, R);

  painter.setBrush(Qt::NoBrush);
  painter.setPen(QPen(accent, 2.0));
  painter.drawEllipse(c, R - 1.5, R - 1.5);

  painter.setPen(Qt::NoPen);
  painter.setBrush(well);
  painter.drawEllipse(c, well_r, well_r);

  const double glyph_r = (well_r + R) * 0.52;
  const double glyph = std::max(5.0, R * 0.08);
  painter.setPen(Qt::NoPen);
  painter.setBrush(accent);
  if (omni) {
    DrawDirTriangle(painter, c, -kPi * 0.5, glyph_r, glyph, accent);
    DrawDirTriangle(painter, c, kPi * 0.5, glyph_r, glyph, accent);
  }
  DrawDirTriangle(painter, c, kPi, glyph_r, glyph, accent);
  DrawDirTriangle(painter, c, 0.0, glyph_r, glyph, accent);

  painter.setBrush(knob);
  painter.setPen(QPen(accent, 1.6));
  painter.drawEllipse(stick, knob_r, knob_r);
}

void TeleopJoystickWidget::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  update();
}

void TeleopJoystickWidget::changeEvent(QEvent* event) {
  QWidget::changeEvent(event);
  if (event == nullptr) {
    return;
  }
  switch (event->type()) {
    case QEvent::StyleChange:
    case QEvent::PaletteChange:
    case QEvent::ApplicationPaletteChange:
      update();
      break;
    default:
      break;
  }
}

void TeleopJoystickWidget::mousePressEvent(QMouseEvent* event) {
  if (event->button() != Qt::LeftButton) {
    event->ignore();
    return;
  }
  if (royal_style_) {
    // Only the knob drives. The rest of the disc lets the HUD overlay drag.
    const QPointF delta = event->position() - center();
    const double drive_r = radius() * kKnobRatio * 1.85;
    if (std::hypot(delta.x(), delta.y()) > drive_r) {
      event->ignore();
      return;
    }
  }
  dragging_ = true;
  setCursor(Qt::ClosedHandCursor);
  grabMouse();
  updateFromPosition(event->position());
  event->accept();
}

void TeleopJoystickWidget::mouseMoveEvent(QMouseEvent* event) {
  if (!dragging_) {
    return;
  }
  updateFromPosition(event->position());
  event->accept();
}

void TeleopJoystickWidget::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() != Qt::LeftButton || !dragging_) {
    return;
  }
  dragging_ = false;
  setCursor(Qt::OpenHandCursor);
  releaseMouse();
  knob_ = QPointF(0.0, 0.0);
  update();
  emit valueChanged(0.0, 0.0);
  emit released();
  event->accept();
}

}  // namespace teleop
}  // namespace autoviz

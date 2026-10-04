/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/teleop/teleop_joystick_widget.hpp"

#include <algorithm>
#include <cmath>

#include <QFontMetrics>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QResizeEvent>

namespace autoviz {
namespace teleop {
namespace {

constexpr double kWellRatio = 0.62;   // thumbstick bowl / outer
constexpr double kKnobRatio = 0.22;   // knob head / outer
constexpr double kTravelRatio = 0.28; // max knob offset / outer
constexpr int kLabelBand = 0;
constexpr int kMinPadSide = 108;

/** Flat colors — no gradients. */
constexpr QColor kRing(0xF0, 0x8C, 0x28);
constexpr QColor kRingActive(0xE0, 0x72, 0x12);
constexpr QColor kSectorPress(0xC2, 0x41, 0x0C);  // pressed direction wedge
constexpr QColor kIcon(255, 255, 255);
constexpr QColor kIconPress(0xFE, 0xF3, 0xC7);    // warm highlight on press
constexpr QColor kWell(0xF1, 0xF5, 0xF9);
constexpr QColor kWellRim(0xCB, 0xD5, 0xE1);
constexpr QColor kKnob(0xFF, 0xFF, 0xFF);
constexpr QColor kKnobRim(0x94, 0xA3, 0xB8);
constexpr QColor kKnobCore(0xF0, 0x8C, 0x28);

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
  setFocusPolicy(Qt::StrongFocus);
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
  const int side =
      std::max(kMinPadSide, std::min(width(), height() - kLabelBand) - 8);
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
  const bool active = dragging_ || std::hypot(knob_.x(), knob_.y()) > 1e-3;
  const bool omni = axes_ == TeleopStickAxes::kOmni;
  constexpr double kPi = 3.14159265358979323846;

  // Knob rests in the bowl and slides — physical thumbstick, no drawn shaft.
  const QPointF stick(c.x() + knob_.x() * travel, c.y() + knob_.y() * travel);

  // Pad shadow
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(15, 23, 42, 26));
  painter.drawEllipse(outer.translated(0, 3));

  // Orange bezel ring
  {
    QPainterPath ring;
    ring.addEllipse(outer);
    QPainterPath hole;
    hole.addEllipse(c, well_r, well_r);
    ring = ring.subtracted(hole);
    painter.fillPath(ring, active ? kRingActive : kRing);
  }

  // Pressed direction sectors (flat color change)
  const bool press_f = omni && knob_.y() < -kPressThreshold;
  const bool press_b = omni && knob_.y() > kPressThreshold;
  const bool press_l = knob_.x() < -kPressThreshold;
  const bool press_r = knob_.x() > kPressThreshold;
  constexpr double kHalfSpan = 0.42;  // ~24°
  painter.setPen(Qt::NoPen);
  if (press_f) {
    DrawDirSector(painter, c, R, well_r, -kPi * 0.5, kHalfSpan, kSectorPress);
  }
  if (press_b) {
    DrawDirSector(painter, c, R, well_r, kPi * 0.5, kHalfSpan, kSectorPress);
  }
  if (press_l) {
    DrawDirSector(painter, c, R, well_r, kPi, kHalfSpan, kSectorPress);
  }
  if (press_r) {
    DrawDirSector(painter, c, R, well_r, 0.0, kHalfSpan, kSectorPress);
  }

  // Direction glyphs — brighter / larger when that way is pressed
  const double glyph_r = (well_r + R) * 0.5;
  const double glyph = std::max(6.5, R * 0.10);
  const auto glyph_color = [](bool pressed) {
    return pressed ? kIconPress : kIcon;
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

  // Recessed bowl (flat)
  {
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(15, 23, 42, 20));
    painter.drawEllipse(c + QPointF(0, 1.5), well_r, well_r);

    painter.setBrush(kWell);
    painter.setPen(QPen(kWellRim, 1.2));
    painter.drawEllipse(c, well_r, well_r);

    // Inner lip — suggests depth without gradient
    painter.setPen(QPen(QColor(148, 163, 184, 90), 1.0));
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(c, well_r - 3.0, well_r - 3.0);
  }

  // Contact shadow under knob (offset opposite tilt = lifts toward viewer)
  {
    const QPointF shadow =
        stick + QPointF(-knob_.x() * 1.5, 2.0 + std::abs(knob_.y()) * 1.2);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(15, 23, 42, active ? 40 : 28));
    painter.drawEllipse(shadow, knob_r * 1.05, knob_r * 0.92);
  }

  // Thumbstick head
  {
    painter.setBrush(kKnob);
    painter.setPen(QPen(active ? kRingActive : kKnobRim, 1.6));
    painter.drawEllipse(stick, knob_r, knob_r);

    // Flat core cue (not a shaft)
    painter.setPen(Qt::NoPen);
    painter.setBrush(active ? kRingActive : kKnobCore);
    painter.drawEllipse(stick, knob_r * 0.34, knob_r * 0.34);
  }
}

void TeleopJoystickWidget::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  update();
}

void TeleopJoystickWidget::mousePressEvent(QMouseEvent* event) {
  if (event->button() != Qt::LeftButton) {
    return;
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

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

constexpr double kKnobRadiusRatio = 0.20;
constexpr double kDeadzoneRatio = 0.10;
constexpr int kLabelBand = 32;
constexpr int kMinPadSide = 120;

}  // namespace

TeleopJoystickWidget::TeleopJoystickWidget(const QString& label, QWidget* parent)
    : label_(label), QWidget(parent) {
  setMinimumSize(160, 190);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setMouseTracking(true);
  setFocusPolicy(Qt::StrongFocus);
  setAttribute(Qt::WA_StyledBackground, true);
}

QSize TeleopJoystickWidget::sizeHint() const { return QSize(200, 220); }

QSize TeleopJoystickWidget::minimumSizeHint() const { return QSize(160, 190); }

QPointF TeleopJoystickWidget::normalizedValue() const { return knob_; }

void TeleopJoystickWidget::setLabel(const QString& label) {
  if (label_ == label) {
    return;
  }
  label_ = label;
  update();
}

void TeleopJoystickWidget::setNormalizedValue(const QPointF& value,
                                              bool emit_signal) {
  const QPointF clamped = clampToUnitDisk(value);
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
  const int available_h = std::max(kMinPadSide, height() - kLabelBand);
  const int side = std::max(kMinPadSide, std::min(width() - 8, available_h));
  const double x = (width() - side) * 0.5;
  const double y = std::max(4.0, (height() - kLabelBand - side) * 0.5);
  return QRectF(x, y, side, side);
}

QPointF TeleopJoystickWidget::center() const {
  return outerRect().center();
}

double TeleopJoystickWidget::radius() const { return outerRect().width() * 0.5; }

QPointF TeleopJoystickWidget::clampToUnitDisk(const QPointF& vector) const {
  const double length = std::hypot(vector.x(), vector.y());
  if (length <= 1.0 || length <= 1e-6) {
    return vector;
  }
  return vector / length;
}

void TeleopJoystickWidget::updateFromPosition(const QPointF& local_pos) {
  const QPointF center_pt = center();
  const double max_radius = radius() * (1.0 - kKnobRadiusRatio - 0.05);
  QPointF delta(local_pos.x() - center_pt.x(), local_pos.y() - center_pt.y());
  if (max_radius > 1e-3) {
    delta /= max_radius;
  }
  const QPointF clamped = clampToUnitDisk(delta);
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
  const QPointF center_pt = outer.center();
  const double outer_r = outer.width() * 0.5;
  const double knob_r = outer_r * kKnobRadiusRatio;
  const bool active = dragging_ || std::hypot(knob_.x(), knob_.y()) > 1e-3;

  // Soft drop shadow under pad
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(15, 23, 42, 28));
  painter.drawEllipse(outer.translated(0.0, 3.0));

  // Pad face — higher contrast than card white
  QRadialGradient base_gradient(center_pt, outer_r);
  base_gradient.setColorAt(0.0, QColor(236, 242, 247));
  base_gradient.setColorAt(0.55, QColor(226, 232, 240));
  base_gradient.setColorAt(1.0, QColor(203, 213, 225));
  painter.setPen(QPen(QColor(100, 116, 139), 2.0));
  painter.setBrush(base_gradient);
  painter.drawEllipse(outer);

  // Inner ring
  painter.setPen(QPen(active ? QColor(8, 145, 178) : QColor(71, 85, 105),
                      active ? 3.0 : 2.0));
  painter.setBrush(Qt::NoBrush);
  painter.drawEllipse(outer.adjusted(5.0, 5.0, -5.0, -5.0));

  // Crosshair
  painter.setPen(QPen(QColor(71, 85, 105, 160), 1.4));
  painter.drawLine(QPointF(center_pt.x() - outer_r * 0.70, center_pt.y()),
                   QPointF(center_pt.x() + outer_r * 0.70, center_pt.y()));
  painter.drawLine(QPointF(center_pt.x(), center_pt.y() - outer_r * 0.70),
                   QPointF(center_pt.x(), center_pt.y() + outer_r * 0.70));

  // Cardinal ticks
  painter.setPen(QPen(QColor(51, 65, 85), 2.0));
  const double tick = outer_r * 0.08;
  painter.drawLine(QPointF(center_pt.x(), center_pt.y() - outer_r * 0.82),
                   QPointF(center_pt.x(), center_pt.y() - outer_r * 0.82 + tick));
  painter.drawLine(QPointF(center_pt.x(), center_pt.y() + outer_r * 0.82 - tick),
                   QPointF(center_pt.x(), center_pt.y() + outer_r * 0.82));
  painter.drawLine(QPointF(center_pt.x() - outer_r * 0.82, center_pt.y()),
                   QPointF(center_pt.x() - outer_r * 0.82 + tick, center_pt.y()));
  painter.drawLine(QPointF(center_pt.x() + outer_r * 0.82 - tick, center_pt.y()),
                   QPointF(center_pt.x() + outer_r * 0.82, center_pt.y()));

  // Deadzone
  const double deadzone_r = outer_r * kDeadzoneRatio;
  painter.setPen(QPen(QColor(100, 116, 139), 1.2, Qt::DotLine));
  painter.setBrush(Qt::NoBrush);
  painter.drawEllipse(center_pt, deadzone_r, deadzone_r);

  const double travel = outer_r * (1.0 - kKnobRadiusRatio - 0.06);
  const QPointF knob_center(center_pt.x() + knob_.x() * travel,
                            center_pt.y() + knob_.y() * travel);

  // Knob shadow
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(15, 23, 42, 50));
  painter.drawEllipse(knob_center + QPointF(0.0, 2.5), knob_r + 1.5, knob_r + 1.5);

  QRadialGradient knob_gradient(knob_center - QPointF(knob_r * 0.3, knob_r * 0.3),
                                knob_r * 1.4);
  if (active) {
    knob_gradient.setColorAt(0.0, QColor(165, 243, 252));
    knob_gradient.setColorAt(0.4, QColor(34, 211, 238));
    knob_gradient.setColorAt(1.0, QColor(8, 145, 178));
  } else {
    knob_gradient.setColorAt(0.0, QColor(125, 211, 252));
    knob_gradient.setColorAt(0.45, QColor(14, 165, 233));
    knob_gradient.setColorAt(1.0, QColor(3, 105, 161));
  }
  painter.setPen(QPen(QColor(255, 255, 255, 220), 1.6));
  painter.setBrush(knob_gradient);
  painter.drawEllipse(knob_center, knob_r, knob_r);

  painter.setPen(QPen(QColor(255, 255, 255, 120), 1.0));
  painter.setBrush(Qt::NoBrush);
  painter.drawEllipse(knob_center, knob_r * 0.5, knob_r * 0.5);

  // Label under pad
  QFont label_font = font();
  label_font.setPointSize(std::max(11, label_font.pointSize() + 1));
  label_font.setWeight(QFont::Bold);
  painter.setFont(label_font);
  painter.setPen(QColor(30, 41, 59));
  const QRect label_rect(0, height() - kLabelBand + 2, width(), kLabelBand - 4);
  painter.drawText(label_rect, Qt::AlignHCenter | Qt::AlignVCenter, label_);
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
  releaseMouse();
  knob_ = QPointF(0.0, 0.0);
  update();
  emit valueChanged(0.0, 0.0);
  emit released();
  event->accept();
}

}  // namespace teleop
}  // namespace autoviz

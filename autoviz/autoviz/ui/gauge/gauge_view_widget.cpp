/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/gauge/gauge_view_widget.hpp"

#include <QPainter>
#include <QPalette>
#include <QResizeEvent>

#include <algorithm>
#include <cmath>

namespace autoviz {
namespace gauge {
namespace {

constexpr int kArcStartDefault = 225 * 16;
constexpr int kArcSpanDefault = -270 * 16;

QColor LerpColor(const QColor& a, const QColor& b, double t) {
  t = std::clamp(t, 0.0, 1.0);
  return QColor(static_cast<int>(a.red() + (b.red() - a.red()) * t),
                static_cast<int>(a.green() + (b.green() - a.green()) * t),
                static_cast<int>(a.blue() + (b.blue() - a.blue()) * t));
}

QColor RainbowColor(double t) {
  t = std::clamp(t, 0.0, 1.0);
  const double hue = (1.0 - t) * 300.0;
  QColor color;
  color.setHsvF(hue / 360.0, 0.85, 0.95);
  return color;
}

QColor TurboColor(double t) {
  t = std::clamp(t, 0.0, 1.0);
  const QColor stops[] = {QColor(48, 18, 59),  QColor(68, 59, 129), QColor(62, 120, 180),
                          QColor(49, 176, 160), QColor(152, 213, 72), QColor(253, 231, 37)};
  const double scaled = t * 5.0;
  const int index = std::min(4, static_cast<int>(scaled));
  return LerpColor(stops[index], stops[index + 1], scaled - index);
}

QColor RedYellowGreenColor(double t) {
  t = std::clamp(t, 0.0, 1.0);
  if (t < 0.5) {
    return LerpColor(QColor(210, 70, 70), QColor(230, 190, 60), t * 2.0);
  }
  return LerpColor(QColor(230, 190, 60), QColor(70, 170, 95), (t - 0.5) * 2.0);
}

QString FormatValue(double value) {
  const double abs_value = std::abs(value);
  if (abs_value >= 1000.0) {
    return QString::number(value, 'g', 5);
  }
  if (abs_value >= 10.0) {
    return QString::number(value, 'f', 1);
  }
  return QString::number(value, 'g', 4);
}

}  // namespace

GaugeViewWidget::GaugeViewWidget(QWidget* parent) : QWidget(parent) {
  setMinimumSize(120, 100);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setAttribute(Qt::WA_StyledBackground, true);
}

void GaugeViewWidget::setConfig(const GaugePanelConfig& config) {
  config_ = config;
  update();
}

void GaugeViewWidget::setValue(double value, bool has_value) {
  value_ = value;
  has_value_ = has_value;
  if (has_value) {
    error_text_.clear();
  }
  update();
}

void GaugeViewWidget::setErrorText(const QString& text) {
  error_text_ = text;
  has_value_ = false;
  update();
}

double GaugeViewWidget::normalizedValue() const {
  const double range = config_.max_value - config_.min_value;
  if (range <= 0.0) {
    return 0.0;
  }
  return std::clamp((value_ - config_.min_value) / range, 0.0, 1.0);
}

int GaugeViewWidget::startAngleSixteenths() const {
  if (config_.reverse_direction) {
    return -45 * 16;
  }
  return kArcStartDefault;
}

int GaugeViewWidget::spanAngleSixteenths() const {
  if (config_.reverse_direction) {
    return 270 * 16;
  }
  return kArcSpanDefault;
}

int GaugeViewWidget::valueSpanSixteenths() const {
  return static_cast<int>(spanAngleSixteenths() * normalizedValue());
}

QColor GaugeViewWidget::colorForNormalizedValue(double t) const {
  if (config_.reverse_color) {
    t = 1.0 - t;
  }
  switch (config_.color_mode) {
    case GaugeColorMode::kSolid:
      return config_.gradient_end.isValid() ? config_.gradient_end
                                            : palette().color(QPalette::Highlight);
    case GaugeColorMode::kGradient:
      return LerpColor(config_.gradient_start, config_.gradient_end, t);
    case GaugeColorMode::kColorMap:
      switch (config_.color_map) {
        case GaugeColorMap::kRainbow:
          return RainbowColor(t);
        case GaugeColorMap::kTurbo:
          return TurboColor(t);
        case GaugeColorMap::kRedYellowGreen:
        default:
          return RedYellowGreenColor(t);
      }
  }
  return palette().color(QPalette::Highlight);
}

void GaugeViewWidget::paintEvent(QPaintEvent* /*event*/) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);

  const QRectF bounds = rect().adjusted(10, 8, -10, -8);
  if (bounds.width() < 40.0 || bounds.height() < 40.0) {
    return;
  }

  if (!error_text_.isEmpty()) {
    painter.setPen(palette().color(QPalette::Mid));
    painter.drawText(bounds, Qt::AlignCenter | Qt::TextWordWrap, error_text_);
    return;
  }

  const QPointF center(bounds.center().x(), bounds.bottom() - 6.0);
  const double radius =
      std::min({bounds.width() * 0.46, bounds.height() * 0.88, 220.0});
  const QRectF arc_rect(center.x() - radius, center.y() - radius, radius * 2.0,
                        radius * 2.0);

  QPen track_pen(palette().color(QPalette::Midlight), 10.0, Qt::SolidLine,
                 Qt::FlatCap);
  painter.setPen(track_pen);
  painter.setBrush(Qt::NoBrush);
  painter.drawArc(arc_rect, startAngleSixteenths(), spanAngleSixteenths());

  if (has_value_) {
    const QColor value_color = colorForNormalizedValue(normalizedValue());
    QPen value_pen(value_color, 10.0, Qt::SolidLine, Qt::FlatCap);
    painter.setPen(value_pen);
    painter.drawArc(arc_rect, startAngleSixteenths(), valueSpanSixteenths());

    const double start_deg = startAngleSixteenths() / 16.0;
    const double span_deg = spanAngleSixteenths() / 16.0;
    const double needle_deg = start_deg + span_deg * normalizedValue();
    const double needle_rad = qDegreesToRadians(needle_deg);
    const QPointF needle_tip(center.x() + std::cos(needle_rad) * radius * 0.88,
                           center.y() - std::sin(needle_rad) * radius * 0.88);

    QPen needle_pen(palette().color(QPalette::Text), 2.5, Qt::SolidLine,
                    Qt::RoundCap);
    painter.setPen(needle_pen);
    painter.drawLine(center, needle_tip);

    painter.setBrush(palette().color(QPalette::Text));
    painter.drawEllipse(center, 4.5, 4.5);

    QFont value_font = painter.font();
    value_font.setPointSizeF(std::max(12.0, radius * 0.18));
    value_font.setBold(true);
    painter.setFont(value_font);
    painter.setPen(palette().color(QPalette::Text));
    const QRectF value_rect(center.x() - radius * 0.55, center.y() - radius * 0.55,
                            radius * 1.1, radius * 0.55);
    painter.drawText(value_rect, Qt::AlignHCenter | Qt::AlignBottom,
                     FormatValue(value_));
  } else {
    painter.setPen(palette().color(QPalette::Mid));
    painter.drawText(bounds, Qt::AlignCenter, tr("Waiting for data"));
  }

  QFont label_font = painter.font();
  label_font.setPointSizeF(std::max(8.0, label_font.pointSizeF() - 1.0));
  painter.setFont(label_font);
  painter.setPen(palette().color(QPalette::Mid));

  const double start_deg = startAngleSixteenths() / 16.0;
  const double span_deg = spanAngleSixteenths() / 16.0;
  const auto label_point = [&](double t) {
    const double deg = start_deg + span_deg * t;
    const double rad = qDegreesToRadians(deg);
    return QPointF(center.x() + std::cos(rad) * radius * 1.02,
                   center.y() - std::sin(rad) * radius * 1.02);
  };

  const QPointF min_pt = label_point(0.0);
  const QPointF max_pt = label_point(1.0);
  painter.drawText(QRectF(min_pt.x() - 28, min_pt.y() - 10, 56, 20),
                   Qt::AlignCenter, FormatValue(config_.min_value));
  painter.drawText(QRectF(max_pt.x() - 28, max_pt.y() - 10, 56, 20),
                   Qt::AlignCenter, FormatValue(config_.max_value));
}

void GaugeViewWidget::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  update();
}

}  // namespace gauge
}  // namespace autoviz

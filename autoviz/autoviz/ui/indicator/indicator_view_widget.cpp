/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/indicator/indicator_view_widget.hpp"

#include <QPainter>
#include <QPalette>
#include <QRadialGradient>

#include <algorithm>

namespace autoviz {
namespace indicator {

IndicatorViewWidget::IndicatorViewWidget(QWidget* parent) : QWidget(parent) {
  setMinimumSize(120, 80);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void IndicatorViewWidget::setConfig(const IndicatorPanelConfig& config) {
  config_ = config;
  update();
}

void IndicatorViewWidget::setMatchResult(const IndicatorMatchResult& result,
                                         bool has_value) {
  if (has_value_ == has_value && result_ == result && error_text_.isEmpty()) {
    return;
  }
  has_value_ = has_value;
  result_ = result;
  error_text_.clear();
  update();
}

void IndicatorViewWidget::setErrorText(const QString& text) {
  if (error_text_ == text) {
    return;
  }
  error_text_ = text;
  has_value_ = false;
  update();
}

void IndicatorViewWidget::paintBulb(QPainter* painter, const QRect& area) {
  const int diameter = std::min(area.width(), area.height() / 2);
  const QRect bulb_rect(area.center().x() - diameter / 2, area.top() + 8, diameter,
                        diameter);
  QColor bulb_color = result_.matched ? result_.color : QColor(120, 120, 130);
  if (!has_value_) {
    bulb_color = palette().color(QPalette::Mid);
  }

  QRadialGradient gradient(bulb_rect.center(), diameter * 0.55);
  gradient.setColorAt(0.0, bulb_color.lighter(140));
  gradient.setColorAt(1.0, bulb_color.darker(115));
  painter->setPen(QPen(bulb_color.darker(130), 1.5));
  painter->setBrush(gradient);
  painter->drawEllipse(bulb_rect);

  painter->setPen(palette().color(QPalette::WindowText));
  QFont label_font = painter->font();
  label_font.setPointSize(std::max(10, label_font.pointSize()));
  label_font.setBold(true);
  painter->setFont(label_font);
  const QRect label_rect(area.left(), bulb_rect.bottom() + 8, area.width(),
                         area.bottom() - bulb_rect.bottom() - 8);
  const QString label =
      error_text_.isEmpty() ? (has_value_ ? result_.label : tr("—")) : error_text_;
  painter->drawText(label_rect, Qt::AlignHCenter | Qt::AlignTop | Qt::TextWordWrap,
                    label);
}

void IndicatorViewWidget::paintBackground(QPainter* painter, const QRect& area) {
  QColor fill = result_.matched ? result_.color : QColor(70, 74, 82);
  if (!has_value_) {
    fill = palette().color(QPalette::Mid);
  }
  painter->fillRect(area, fill);

  painter->setPen(fill.lightness() > 128 ? QColor(20, 20, 24) : QColor(245, 245, 248));
  QFont label_font = painter->font();
  label_font.setPointSize(std::max(14, label_font.pointSize() + 2));
  label_font.setBold(true);
  painter->setFont(label_font);
  const QString label =
      error_text_.isEmpty() ? (has_value_ ? result_.label : tr("—")) : error_text_;
  painter->drawText(area.adjusted(12, 12, -12, -12),
                    Qt::AlignCenter | Qt::TextWordWrap, label);
}

void IndicatorViewWidget::paintEvent(QPaintEvent* /*event*/) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), palette().color(QPalette::Window));

  if (config_.style == IndicatorStyle::kBackground) {
    paintBackground(&painter, rect());
  } else {
    paintBulb(&painter, rect());
  }
}

}  // namespace indicator
}  // namespace autoviz

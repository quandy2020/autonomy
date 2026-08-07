/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/autoviz_proxy_style.hpp"

#include <QApplication>
#include <QPainter>
#include <QStyle>
#include <QStyleFactory>
#include <QStyleOption>

namespace autoviz {
namespace {

constexpr int kScrollBarExtent = 6;

}  // namespace

AutovizProxyStyle::AutovizProxyStyle(QStyle* base_style)
    : QProxyStyle(base_style != nullptr ? base_style
                                        : QStyleFactory::create(QStringLiteral("Fusion"))) {}

void AutovizProxyStyle::setAccentColor(const QColor& accent) { accent_ = accent; }

void AutovizProxyStyle::setSelectionBackground(const QColor& color) {
  selection_bg_ = color;
}

void AutovizProxyStyle::setHoverBackground(const QColor& color) { hover_bg_ = color; }

void AutovizProxyStyle::drawPrimitive(PrimitiveElement element,
                                      const QStyleOption* option, QPainter* painter,
                                      const QWidget* widget) const {
  switch (element) {
    case PE_IndicatorCheckBox:
    case PE_IndicatorRadioButton:
    case PE_PanelItemViewItem:
    case PE_FrameFocusRect:
      QProxyStyle::drawPrimitive(element, option, painter, widget);
      return;
    default:
      break;
  }
  QProxyStyle::drawPrimitive(element, option, painter, widget);
}

void AutovizProxyStyle::drawControl(ControlElement element, const QStyleOption* option,
                                    QPainter* painter, const QWidget* widget) const {
  QProxyStyle::drawControl(element, option, painter, widget);
}

int AutovizProxyStyle::pixelMetric(PixelMetric metric, const QStyleOption* option,
                                   const QWidget* widget) const {
  switch (metric) {
    case PM_ScrollBarExtent:
      return kScrollBarExtent;
    case PM_TabBarTabHSpace:
      return 12;
    case PM_TabBarTabVSpace:
      return 6;
    default:
      break;
  }
  return QProxyStyle::pixelMetric(metric, option, widget);
}

}  // namespace autoviz

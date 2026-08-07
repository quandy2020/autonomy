/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QProxyStyle>

class QPainter;
class QStyleOption;
class QStyleOptionTab;
class QWidget;

namespace autoviz {

/**
 * Fusion proxy — scroll metrics and palette hooks; Panels menu checkboxes use QSS.
 */
class AutovizProxyStyle : public QProxyStyle {
  Q_OBJECT

 public:
  explicit AutovizProxyStyle(QStyle* base_style = nullptr);

  void setAccentColor(const QColor& accent);
  void setSelectionBackground(const QColor& color);
  void setHoverBackground(const QColor& color);

  void drawPrimitive(PrimitiveElement element, const QStyleOption* option,
                     QPainter* painter, const QWidget* widget = nullptr) const override;

  void drawControl(ControlElement element, const QStyleOption* option,
                   QPainter* painter, const QWidget* widget = nullptr) const override;

  int pixelMetric(PixelMetric metric, const QStyleOption* option = nullptr,
                  const QWidget* widget = nullptr) const override;

 private:
  QColor accent_{0x30, 0x8C, 0xC6};
  QColor selection_bg_{0x30, 0x8C, 0xC6};
  QColor hover_bg_{0xE5, 0xF3, 0xFF};
};

}  // namespace autoviz

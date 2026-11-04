/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QAbstractButton>
#include <QColor>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QPaintEvent;

namespace autoviz::widget {

// Compact color input with a native QColorDialog editor. The current color is
// always exposed as text for users who cannot distinguish the swatch.
class ColorPicker final : public QAbstractButton {
  Q_OBJECT
  Q_PROPERTY(QColor color READ color WRITE SetColor NOTIFY ColorChanged)

 public:
  explicit ColorPicker(QWidget* parent = nullptr);

  QColor color() const noexcept { return color_; }
  void SetColor(const QColor& color);
  bool alpha_enabled() const noexcept { return alpha_enabled_; }
  void SetAlphaEnabled(bool enabled);
  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 signals:
  void ColorChanged(const QColor& color);

 protected:
  void paintEvent(QPaintEvent* event) override;

 private slots:
  void OpenEditor();

 private:
  void UpdateAccessibility();

  QColor color_ = QColor(Qt::white);
  style::Tone tone_ = style::Tone::Shell;
  bool alpha_enabled_ = false;
};

}  // namespace autoviz::widget

/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QDoubleSpinBox>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QEvent;
class QPaintEvent;
class QWheelEvent;

namespace autoviz::widget {

// Numeric parameter editor that keeps native QDoubleSpinBox semantics and
// prevents accidental value changes while a containing panel is scrolled.
class NumberField final : public QDoubleSpinBox {
 public:
  explicit NumberField(QWidget* parent = nullptr);

  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);
  void SetWheelRequiresFocus(bool requires_focus);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 protected:
  bool event(QEvent* event) override;
  bool eventFilter(QObject* watched, QEvent* event) override;
  void paintEvent(QPaintEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;

 private:
  style::Tone tone_ = style::Tone::Shell;
  bool wheel_requires_focus_ = true;
};

}  // namespace autoviz::widget

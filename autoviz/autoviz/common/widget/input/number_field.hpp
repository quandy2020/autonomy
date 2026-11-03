/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QDoubleSpinBox>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

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
  void wheelEvent(QWheelEvent* event) override;

 private:
  style::Tone tone_ = style::Tone::Shell;
  bool wheel_requires_focus_ = true;
};

}  // namespace autoviz::widget

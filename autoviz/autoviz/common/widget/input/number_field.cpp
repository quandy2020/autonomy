/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/input/number_field.hpp"

#include <QVariant>
#include <QWheelEvent>

#include "autoviz/common/widget/foundation/widget_theme.hpp"

namespace autoviz::widget {

NumberField::NumberField(QWidget* parent) : QDoubleSpinBox(parent) {
  setObjectName(QStringLiteral("AutovizNumberField"));
  setProperty("autovizRole", QStringLiteral("numberField"));
  setFocusPolicy(Qt::StrongFocus);
  setKeyboardTracking(false);
  setStepType(QAbstractSpinBox::AdaptiveDecimalStepType);
  setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  setPalette(BuildWidgetPalette(tone_, palette()));
}

void NumberField::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  setPalette(BuildWidgetPalette(tone_, palette()));
  update();
}

void NumberField::SetWheelRequiresFocus(bool requires_focus) {
  wheel_requires_focus_ = requires_focus;
}

QSize NumberField::sizeHint() const {
  QSize hint = QDoubleSpinBox::sizeHint();
  hint.setWidth(qMax(hint.width(), 112));
  hint.setHeight(GetThemeTokens(tone_).control_height_medium);
  return hint;
}

QSize NumberField::minimumSizeHint() const {
  QSize hint = QDoubleSpinBox::minimumSizeHint();
  hint.setHeight(GetThemeTokens(tone_).control_height_small);
  return hint;
}

void NumberField::wheelEvent(QWheelEvent* event) {
  if (wheel_requires_focus_ && !hasFocus()) {
    event->ignore();
    return;
  }
  QDoubleSpinBox::wheelEvent(event);
}

}  // namespace autoviz::widget

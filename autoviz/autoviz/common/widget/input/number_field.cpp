/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/input/number_field.hpp"

#include <QEvent>
#include <QLineEdit>
#include <QPaintEvent>
#include <QPainter>
#include <QVariant>
#include <QWheelEvent>

#include "autoviz/common/widget/foundation/validation_state.hpp"
#include "autoviz/common/widget/foundation/widget_theme.hpp"

namespace autoviz::widget {
namespace {

void EnsureTransparentEditorBase(QWidget* widget) {
  QPalette palette = widget->palette();
  bool changed = false;
  for (const QPalette::ColorGroup group :
       {QPalette::Active, QPalette::Inactive, QPalette::Disabled}) {
    if (palette.brush(group, QPalette::Base).color().alpha() != 0) {
      palette.setBrush(group, QPalette::Base, Qt::transparent);
      changed = true;
    }
  }
  if (changed) {
    widget->setPalette(palette);
  }
}

}  // namespace

NumberField::NumberField(QWidget* parent) : QDoubleSpinBox(parent) {
  setObjectName(QStringLiteral("AutovizNumberField"));
  setProperty("autovizRole", QStringLiteral("numberField"));
  setFocusPolicy(Qt::StrongFocus);
  setKeyboardTracking(false);
  setStepType(QAbstractSpinBox::AdaptiveDecimalStepType);
  setButtonSymbols(QAbstractSpinBox::NoButtons);
  setFrame(false);
  setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  setAttribute(Qt::WA_Hover);
  setAttribute(Qt::WA_TranslucentBackground);
  setPalette(BuildWidgetPalette(tone_, palette()));
  EnsureTransparentEditorBase(this);
  if (QLineEdit* editor = lineEdit()) {
    editor->setFrame(false);
    editor->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    editor->setTextMargins(12, 0, 12, 0);
    editor->setAttribute(Qt::WA_TranslucentBackground);
    EnsureTransparentEditorBase(editor);
    editor->installEventFilter(this);
  }
}

void NumberField::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  setPalette(BuildWidgetPalette(tone_, palette()));
  EnsureTransparentEditorBase(this);
  if (QLineEdit* editor = lineEdit()) {
    editor->setPalette(BuildWidgetPalette(tone_, editor->palette()));
    EnsureTransparentEditorBase(editor);
  }
  update();
}

void NumberField::SetWheelRequiresFocus(bool requires_focus) {
  wheel_requires_focus_ = requires_focus;
}

QSize NumberField::sizeHint() const {
  QSize hint = QDoubleSpinBox::sizeHint();
  hint.setWidth(qMax(hint.width(), 112));
  hint.setHeight(GetThemeTokens(this, tone_).control_height_medium);
  return hint;
}

QSize NumberField::minimumSizeHint() const {
  QSize hint = QDoubleSpinBox::minimumSizeHint();
  hint.setHeight(GetThemeTokens(this, tone_).control_height_small);
  return hint;
}

bool NumberField::event(QEvent* event) {
  switch (event->type()) {
    case QEvent::Enter:
    case QEvent::Leave:
    case QEvent::FocusIn:
    case QEvent::FocusOut:
    case QEvent::EnabledChange:
    case QEvent::DynamicPropertyChange:
      update();
      break;
    case QEvent::PaletteChange:
      EnsureTransparentEditorBase(this);
      if (QLineEdit* editor = lineEdit()) {
        EnsureTransparentEditorBase(editor);
      }
      update();
      break;
    default:
      break;
  }
  return QDoubleSpinBox::event(event);
}

bool NumberField::eventFilter(QObject* watched, QEvent* event) {
  if (watched == lineEdit()) {
    switch (event->type()) {
      case QEvent::Enter:
      case QEvent::Leave:
      case QEvent::FocusIn:
      case QEvent::FocusOut:
      case QEvent::EnabledChange:
        update();
        break;
      case QEvent::PaletteChange:
        EnsureTransparentEditorBase(lineEdit());
        update();
        break;
      default:
        break;
    }
  }
  return QDoubleSpinBox::eventFilter(watched, event);
}

void NumberField::paintEvent(QPaintEvent* event) {
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  QColor fill = tokens.surface_sunken;
  QColor border = tokens.border_subtle;
  qreal border_width = 1.0;
  if (!isEnabled()) {
    border = tokens.disabled;
  } else if (internal::HasValidationError(this)) {
    border = tokens.danger;
    border_width = hasFocus() ? tokens.focus_ring_width + 1.0 : 2.0;
  } else if (hasFocus()) {
    border = tokens.focus_ring;
    border_width = tokens.focus_ring_width;
  } else if (underMouse()) {
    fill = tokens.surface_material;
  }

  QPainter background(this);
  background.setRenderHint(QPainter::Antialiasing);
  background.setPen(Qt::NoPen);
  background.setBrush(fill);
  background.drawRoundedRect(QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5),
                             tokens.radius_small, tokens.radius_small);
  background.end();

  QDoubleSpinBox::paintEvent(event);

  QPainter outline(this);
  outline.setRenderHint(QPainter::Antialiasing);
  outline.setBrush(Qt::NoBrush);
  outline.setPen(
      QPen(border, border_width, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  const qreal inset = border_width * 0.5;
  outline.drawRoundedRect(QRectF(rect()).adjusted(inset, inset, -inset, -inset),
                          tokens.radius_small, tokens.radius_small);
}

void NumberField::wheelEvent(QWheelEvent* event) {
  if (wheel_requires_focus_ && !hasFocus()) {
    event->ignore();
    return;
  }
  QDoubleSpinBox::wheelEvent(event);
}

}  // namespace autoviz::widget

/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/feedback/progress_bar.hpp"

#include <QEvent>
#include <QVariant>

namespace autoviz::widget {

ProgressBar::ProgressBar(QWidget* parent) : QProgressBar(parent) {
  setObjectName(QStringLiteral("AutovizProgressBar"));
  setProperty("autovizRole", QStringLiteral("progressBar"));
  setAccessibleName(tr("Progress"));
  setTextVisible(true);
  ApplyPalette();
}

void ProgressBar::SetState(State state) {
  if (state_ == state) {
    return;
  }
  state_ = state;
  ApplyPalette();
}

void ProgressBar::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  ApplyPalette();
}

QSize ProgressBar::sizeHint() const {
  QSize hint = QProgressBar::sizeHint();
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  if (orientation() == Qt::Horizontal) {
    hint.setWidth(qMax(hint.width(), 160));
    hint.setHeight(qMax(hint.height(), tokens.control_height_compact));
  } else {
    hint.setWidth(qMax(hint.width(), tokens.control_height_compact));
    hint.setHeight(qMax(hint.height(), 160));
  }
  return hint;
}

QSize ProgressBar::minimumSizeHint() const {
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  return orientation() == Qt::Horizontal
             ? QSize(64, tokens.control_height_compact)
             : QSize(tokens.control_height_compact, 64);
}

void ProgressBar::changeEvent(QEvent* event) {
  QProgressBar::changeEvent(event);
  if (event->type() == QEvent::EnabledChange ||
      event->type() == QEvent::StyleChange ||
      event->type() == QEvent::ApplicationPaletteChange) {
    ApplyPalette();
  }
}

void ProgressBar::ApplyPalette() {
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  QColor progress_color = tokens.accent;
  switch (state_) {
    case State::kNormal:
      break;
    case State::kSuccess:
      progress_color = tokens.success;
      break;
    case State::kWarning:
      progress_color = tokens.warning;
      break;
    case State::kError:
      progress_color = tokens.danger;
      break;
  }
  QPalette themed = palette();
  themed.setColor(QPalette::Base, tokens.surface_raised);
  themed.setColor(QPalette::Window, tokens.surface_raised);
  themed.setColor(QPalette::Highlight,
                  isEnabled() ? progress_color : tokens.disabled);
  themed.setColor(QPalette::HighlightedText, tokens.on_accent);
  themed.setColor(QPalette::Text, tokens.text);
  setPalette(themed);
  update();
}

}  // namespace autoviz::widget

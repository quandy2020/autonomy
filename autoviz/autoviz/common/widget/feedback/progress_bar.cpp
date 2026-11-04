/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/feedback/progress_bar.hpp"

#include <QEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QVariant>

namespace autoviz::widget {
namespace {

QColor ProgressColor(ProgressBar::State state, const ThemeTokens& tokens) {
  switch (state) {
    case ProgressBar::State::kNormal:
      return tokens.accent;
    case ProgressBar::State::kSuccess:
      return tokens.success;
    case ProgressBar::State::kWarning:
      return tokens.warning;
    case ProgressBar::State::kError:
      return tokens.danger;
  }
  return tokens.accent;
}

}  // namespace

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
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
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
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
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

void ProgressBar::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  QColor progress_color = ProgressColor(state_, tokens);
  QColor track_color = tokens.surface_sunken;
  QColor label_color = tokens.text_secondary;
  if (!isEnabled()) {
    progress_color.setAlpha(qRound(progress_color.alpha() * 0.42));
    track_color.setAlpha(qRound(track_color.alpha() * 0.62));
    label_color = tokens.disabled;
  }

  const bool busy = maximum() <= minimum();
  const qreal fraction =
      busy ? 0.35
           : qBound(0.0,
                    static_cast<qreal>(value() - minimum()) /
                        static_cast<qreal>(maximum() - minimum()),
                    1.0);

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(Qt::NoPen);
  painter.setBrush(track_color);

  QRectF track;
  if (orientation() == Qt::Horizontal) {
    const qreal thickness = isTextVisible() ? 4.0 : 6.0;
    const qreal y = isTextVisible() ? height() - thickness - 2.0
                                    : (height() - thickness) * 0.5;
    track = QRectF(0.5, y, qMax(0.0, width() - 1.0), thickness);
  } else {
    constexpr qreal kThickness = 6.0;
    track = QRectF((width() - kThickness) * 0.5, 0.5, kThickness,
                   qMax(0.0, height() - 1.0));
  }
  const qreal track_radius = qMin(track.width(), track.height()) * 0.5;
  painter.drawRoundedRect(track, track_radius, track_radius);

  QRectF progress_rect = track;
  if (orientation() == Qt::Horizontal) {
    const qreal length = track.width() * fraction;
    const bool reverse =
        invertedAppearance() != (layoutDirection() == Qt::RightToLeft);
    if (busy) {
      progress_rect.setLeft(track.left() + (track.width() - length) * 0.5);
      progress_rect.setWidth(length);
    } else if (reverse) {
      progress_rect.setLeft(track.right() - length);
    } else {
      progress_rect.setWidth(length);
    }
  } else {
    const qreal length = track.height() * fraction;
    if (busy) {
      progress_rect.setTop(track.top() + (track.height() - length) * 0.5);
      progress_rect.setHeight(length);
    } else if (invertedAppearance()) {
      progress_rect.setHeight(length);
    } else {
      progress_rect.setTop(track.bottom() - length);
    }
  }

  if (fraction > 0.0 && progress_rect.width() > 0.0 &&
      progress_rect.height() > 0.0) {
    const qreal progress_radius =
        qMin(progress_rect.width(), progress_rect.height()) * 0.5;
    painter.setBrush(progress_color);
    painter.drawRoundedRect(progress_rect, progress_radius, progress_radius);
  }

  if (isTextVisible() && !text().isEmpty()) {
    painter.setPen(label_color);
    if (orientation() == Qt::Horizontal) {
      painter.drawText(QRectF(0.0, 0.0, width(), qMax(0.0, track.top() - 2.0)),
                       Qt::AlignHCenter | Qt::AlignVCenter, text());
    } else {
      painter.drawText(rect(), Qt::AlignCenter, text());
    }
  }
}

void ProgressBar::ApplyPalette() {
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  const QColor progress_color = ProgressColor(state_, tokens);
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

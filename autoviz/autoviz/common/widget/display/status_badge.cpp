/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/display/status_badge.hpp"

#include <QColor>
#include <QEvent>
#include <QFont>
#include <QFontMetrics>
#include <QPaintEvent>
#include <QPainter>
#include <QResizeEvent>
#include <algorithm>

namespace autoviz {
namespace widget {
namespace {

QColor Blend(const QColor& first, const QColor& second, int alpha) {
  alpha = std::clamp(alpha, 0, 255);
  const int inverse = 255 - alpha;
  return QColor((first.red() * inverse + second.red() * alpha) / 255,
                (first.green() * inverse + second.green() * alpha) / 255,
                (first.blue() * inverse + second.blue() * alpha) / 255,
                (first.alpha() * inverse + second.alpha() * alpha) / 255);
}

QColor AccentForStatus(StatusBadge::Status status, const ThemeTokens& tokens) {
  switch (status) {
    case StatusBadge::Status::kNeutral:
      return tokens.text_muted;
    case StatusBadge::Status::kInformation:
      return tokens.accent;
    case StatusBadge::Status::kSuccess:
      return tokens.success;
    case StatusBadge::Status::kWarning:
      return tokens.warning;
    case StatusBadge::Status::kError:
      return tokens.danger;
  }
  return tokens.text;
}

}  // namespace

StatusBadge::StatusBadge(QWidget* parent)
    : StatusBadge(QString(), Status::kNeutral, parent) {}

StatusBadge::StatusBadge(const QString& text, Status status, QWidget* parent)
    : QWidget(parent), text_(text), status_(status) {
  setObjectName(QStringLiteral("AutovizStatusBadge"));
  setProperty("autovizRole", QStringLiteral("statusBadge"));
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  QFont badge_font = font();
  badge_font.setWeight(QFont::DemiBold);
  setFont(badge_font);
  setAccessibleName(text_);
  UpdateAccessibilityDescription();
  RefreshElisionState();
}

QString StatusBadge::text() const { return text_; }

StatusBadge::Status StatusBadge::status() const { return status_; }

void StatusBadge::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  update();
}

QSize StatusBadge::sizeHint() const {
  const QFontMetrics metrics(font());
  return QSize(metrics.horizontalAdvance(text_) + 18,
               std::max(22, metrics.height() + 8));
}

QSize StatusBadge::minimumSizeHint() const {
  const QFontMetrics metrics(font());
  const int minimum_text_width =
      text_.isEmpty() ? 0 : metrics.horizontalAdvance(QStringLiteral("…"));
  return QSize(minimum_text_width + 18, std::max(22, metrics.height() + 8));
}

void StatusBadge::SetText(const QString& text) {
  if (text_ == text) {
    return;
  }
  text_ = text;
  setAccessibleName(text_);
  updateGeometry();
  RefreshElisionState();
  update();
  emit TextChanged(text_);
}

void StatusBadge::SetStatus(Status status) {
  if (status_ == status) {
    return;
  }
  status_ = status;
  UpdateAccessibilityDescription();
  update();
  emit StatusChanged(status_);
}

void StatusBadge::changeEvent(QEvent* event) {
  QWidget::changeEvent(event);
  if (event != nullptr && event->type() == QEvent::FontChange) {
    updateGeometry();
    RefreshElisionState();
    update();
  }
}

void StatusBadge::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);

  const ThemeTokens& tokens = GetThemeTokens(tone_);
  QColor accent = AccentForStatus(status_, tokens);
  if (!isEnabled()) {
    accent = tokens.disabled;
  }
  const QColor background = Blend(tokens.surface, accent, 33);
  const QColor border = Blend(tokens.surface, accent, 107);

  const QRectF bounds = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
  painter.setPen(QPen(border, 1.0));
  painter.setBrush(background);
  painter.drawRoundedRect(bounds, bounds.height() / 2.0, bounds.height() / 2.0);

  painter.setPen(accent);
  painter.setFont(font());
  const QRect text_bounds = rect().adjusted(9, 0, -9, 0);
  const QString visible_text = QFontMetrics(font()).elidedText(
      text_, Qt::ElideRight, std::max(0, text_bounds.width()));
  painter.drawText(text_bounds, Qt::AlignCenter | Qt::TextSingleLine,
                   visible_text);
}

void StatusBadge::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  RefreshElisionState();
}

void StatusBadge::RefreshElisionState() {
  const int available_width = std::max(0, width() - 18);
  const bool is_elided =
      !text_.isEmpty() &&
      QFontMetrics(font()).horizontalAdvance(text_) > available_width;

  const QString current_tool_tip = toolTip();
  const bool has_custom_tool_tip =
      !current_tool_tip.isEmpty() && (automatic_tool_tip_.isEmpty() ||
                                      current_tool_tip != automatic_tool_tip_);
  if (has_custom_tool_tip) {
    automatic_tool_tip_.clear();
    return;
  }

  if (is_elided) {
    automatic_tool_tip_ = text_;
    if (current_tool_tip != automatic_tool_tip_) {
      setToolTip(automatic_tool_tip_);
    }
    return;
  }

  if (!automatic_tool_tip_.isEmpty() &&
      current_tool_tip == automatic_tool_tip_) {
    setToolTip(QString());
  }
  automatic_tool_tip_.clear();
}

void StatusBadge::UpdateAccessibilityDescription() {
  switch (status_) {
    case Status::kNeutral:
      setAccessibleDescription(tr("Neutral status"));
      break;
    case Status::kInformation:
      setAccessibleDescription(tr("Information status"));
      break;
    case Status::kSuccess:
      setAccessibleDescription(tr("Success status"));
      break;
    case Status::kWarning:
      setAccessibleDescription(tr("Warning status"));
      break;
    case Status::kError:
      setAccessibleDescription(tr("Error status"));
      break;
  }
}

}  // namespace widget
}  // namespace autoviz

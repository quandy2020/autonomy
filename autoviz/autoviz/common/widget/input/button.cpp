/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/input/button.hpp"

#include <QEnterEvent>
#include <QFontMetrics>
#include <QPaintEvent>
#include <QPainter>

namespace autoviz::widget {
namespace {

QColor Mix(const QColor& foreground, const QColor& background, int alpha) {
  const int inverse = 255 - alpha;
  return QColor(
      (foreground.red() * alpha + background.red() * inverse) / 255,
      (foreground.green() * alpha + background.green() * inverse) / 255,
      (foreground.blue() * alpha + background.blue() * inverse) / 255);
}

int MnemonicTextWidth(const QFontMetrics& metrics, const QString& text) {
  constexpr int kMeasurementWidth = 100000;
  const QRect bounds = metrics.boundingRect(
      QRect(0, 0, kMeasurementWidth, metrics.height()),
      Qt::AlignLeft | Qt::AlignVCenter | Qt::TextShowMnemonic, text);
  return bounds.width();
}

}  // namespace

Button::Button(QWidget* parent) : QAbstractButton(parent) {
  setAttribute(Qt::WA_Hover);
  setCursor(Qt::PointingHandCursor);
  setFocusPolicy(Qt::StrongFocus);
}

Button::Button(const QString& text, QWidget* parent) : Button(parent) {
  setText(text);
}

void Button::SetVariant(Variant variant) {
  if (variant_ == variant) return;
  variant_ = variant;
  update();
}

void Button::SetButtonSize(Size size) {
  if (size_ == size) return;
  size_ = size;
  updateGeometry();
  update();
}

void Button::SetTone(style::Tone tone) {
  if (tone_ == tone) return;
  tone_ = tone;
  update();
}

int Button::ControlHeight(const ThemeTokens& tokens) const noexcept {
  switch (size_) {
    case Size::kSmall:
      return tokens.control_height_small;
    case Size::kLarge:
      return tokens.control_height_large;
    case Size::kMedium:
      return tokens.control_height_medium;
  }
  return tokens.control_height_medium;
}

QSize Button::sizeHint() const {
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  const int height = ControlHeight(tokens);
  const int icon_width = icon().isNull() ? 0 : height - tokens.spacing_md;
  const int gap = icon_width > 0 && !text().isEmpty() ? tokens.spacing_sm : 0;
  const int horizontal_padding =
      size_ == Size::kSmall ? tokens.spacing_md : tokens.spacing_lg;
  const int width = MnemonicTextWidth(fontMetrics(), text()) + icon_width +
                    gap + 2 * horizontal_padding;
  return QSize(qMax(height, width), height);
}

QSize Button::minimumSizeHint() const { return sizeHint(); }

void Button::enterEvent(QEnterEvent* event) {
  update();
  QAbstractButton::enterEvent(event);
}

void Button::leaveEvent(QEvent* event) {
  update();
  QAbstractButton::leaveEvent(event);
}

void Button::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  QColor fill;
  QColor foreground;
  QColor border = Qt::transparent;

  switch (variant_) {
    case Variant::kPrimary:
      fill = tokens.accent;
      foreground = tokens.on_accent;
      break;
    case Variant::kDanger:
      fill = tokens.danger;
      foreground = QColor(Qt::white);
      break;
    case Variant::kSubtle:
      fill = Qt::transparent;
      foreground = tokens.text;
      break;
    case Variant::kSecondary:
      fill = tokens.surface_raised;
      foreground = tokens.text;
      border = tokens.border;
      break;
  }

  if (!isEnabled()) {
    foreground = tokens.disabled;
    fill = Mix(tokens.disabled, tokens.surface, 22);
    border = Mix(tokens.disabled, tokens.surface, 80);
  } else if (isDown()) {
    fill = fill == Qt::transparent ? tokens.surface_hovered : fill.darker(108);
  } else if (underMouse()) {
    fill = fill == Qt::transparent ? tokens.surface_hovered : fill.lighter(105);
  }

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setBrush(fill);
  painter.setPen(QPen(border, 1));
  painter.drawRoundedRect(QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5),
                          tokens.radius_small, tokens.radius_small);

  QRect content = rect().adjusted(tokens.spacing_md, 0, -tokens.spacing_md, 0);
  const int icon_extent = qMax(0, height() - tokens.spacing_md);
  const bool has_icon = !icon().isNull();
  const bool has_text = !text().isEmpty();
  int total_width = has_text ? MnemonicTextWidth(fontMetrics(), text()) : 0;
  if (has_icon) total_width += icon_extent;
  if (has_icon && has_text) total_width += tokens.spacing_sm;
  int x = content.center().x() - total_width / 2;

  if (has_icon) {
    const QRect icon_rect(x, (height() - icon_extent) / 2, icon_extent,
                          icon_extent);
    icon().paint(&painter, icon_rect, Qt::AlignCenter,
                 isEnabled() ? QIcon::Normal : QIcon::Disabled);
    x += icon_extent + (has_text ? tokens.spacing_sm : 0);
  }
  if (has_text) {
    painter.setPen(foreground);
    painter.drawText(QRect(x, 0, content.right() - x + 1, height()),
                     Qt::AlignVCenter | Qt::AlignLeft | Qt::TextShowMnemonic,
                     text());
  }

  if (hasFocus()) {
    painter.setBrush(Qt::NoBrush);
    painter.setPen(QPen(tokens.focus_ring, tokens.focus_ring_width));
    const qreal inset = tokens.focus_ring_width + 0.5;
    painter.drawRoundedRect(
        QRectF(rect()).adjusted(inset, inset, -inset, -inset),
        tokens.radius_small - 1, tokens.radius_small - 1);
  }
}

IconButton::IconButton(QWidget* parent) : Button(parent) {
  SetVariant(Variant::kSubtle);
}

IconButton::IconButton(const QIcon& icon, const QString& accessible_name,
                       QWidget* parent)
    : IconButton(parent) {
  setIcon(icon);
  setAccessibleName(accessible_name);
  setToolTip(accessible_name);
}

QSize IconButton::sizeHint() const {
  const QSize base = Button::sizeHint();
  return QSize(base.height(), base.height());
}

}  // namespace autoviz::widget

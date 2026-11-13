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

int MnemonicTextWidth(const QFontMetrics& metrics, const QString& text) {
  constexpr int kMeasurementWidth = 100000;
  const QRect bounds = metrics.boundingRect(
      QRect(0, 0, kMeasurementWidth, metrics.height()),
      Qt::AlignLeft | Qt::AlignVCenter | Qt::TextShowMnemonic, text);
  return bounds.width();
}

QFont ButtonFont(const QFont& base, Button::Variant variant) {
  QFont result = base;
  if (variant == Button::Variant::kPrimary ||
      variant == Button::Variant::kDanger) {
    result.setWeight(QFont::DemiBold);
  }
  return result;
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
  updateGeometry();
  update();
}

void Button::SetButtonSize(Size size) {
  if (size_ == size) return;
  size_ = size;
  updateGeometry();
  update();
}

void Button::SetContentAlign(ContentAlign align) {
  if (content_align_ == align) {
    return;
  }
  content_align_ = align;
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
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  const int height = ControlHeight(tokens);
  const int icon_width = icon().isNull() ? 0 : height - tokens.spacing_md;
  const int gap = icon_width > 0 && !text().isEmpty() ? tokens.spacing_sm : 0;
  const int horizontal_padding =
      size_ == Size::kSmall ? tokens.spacing_md : tokens.spacing_lg;
  const QFontMetrics metrics(ButtonFont(font(), variant_));
  const int width = MnemonicTextWidth(metrics, text()) + icon_width + gap +
                    2 * horizontal_padding;
  return QSize(qMax(height, width), height);
}

QSize Button::minimumSizeHint() const { return sizeHint(); }

void Button::changeEvent(QEvent* event) {
  if (event->type() == QEvent::EnabledChange) {
    setCursor(isEnabled() ? Qt::PointingHandCursor : Qt::ArrowCursor);
    update();
  }
  QAbstractButton::changeEvent(event);
}

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
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  QColor fill;
  QColor foreground;
  QColor border = Qt::transparent;

  switch (variant_) {
    case Variant::kPrimary:
      fill = tokens.accent_filled;
      foreground = tokens.on_accent_filled;
      break;
    case Variant::kDanger:
      fill = tokens.danger_filled;
      foreground = tokens.on_danger_filled;
      break;
    case Variant::kSubtle:
      fill = Qt::transparent;
      // Disclosure headers: muted when collapsed, accent when expanded.
      if (isCheckable()) {
        foreground = isChecked() ? tokens.accent_text : tokens.text_muted;
      } else {
        foreground = tokens.accent_text;
      }
      break;
    case Variant::kSecondary:
      fill = tokens.surface_material;
      foreground = tokens.text;
      break;
  }

  if (!isEnabled()) {
    foreground = tokens.disabled;
    fill =
        variant_ == Variant::kSubtle ? Qt::transparent : tokens.surface_sunken;
    border = Qt::transparent;
  } else if (isDown()) {
    switch (variant_) {
      case Variant::kPrimary:
        fill = tokens.accent_filled_pressed;
        foreground = tokens.on_accent_filled;
        break;
      case Variant::kDanger:
        fill = tokens.danger_filled_pressed;
        foreground = tokens.on_danger_filled;
        break;
      case Variant::kSecondary:
      case Variant::kSubtle:
        fill = tokens.surface_pressed;
        break;
    }
  } else if (underMouse()) {
    switch (variant_) {
      case Variant::kPrimary:
        fill = tokens.accent_filled_hovered;
        foreground = tokens.on_accent_filled;
        break;
      case Variant::kDanger:
        fill = tokens.danger_filled_hovered;
        foreground = tokens.on_danger_filled;
        break;
      case Variant::kSecondary:
      case Variant::kSubtle:
        fill = tokens.surface_hovered;
        if (variant_ == Variant::kSubtle) {
          foreground = tokens.accent_text;
        }
        break;
    }
  }

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setFont(ButtonFont(font(), variant_));
  painter.setBrush(fill);
  painter.setPen(QPen(border, 1));
  painter.drawRoundedRect(QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5),
                          tokens.radius_small, tokens.radius_small);

  const int pad =
      size_ == Size::kSmall ? tokens.spacing_sm : tokens.spacing_md;
  QRect content = rect().adjusted(pad, 0, -pad, 0);
  const int icon_extent =
      qMax(0, qMin(height() - tokens.spacing_sm, tokens.spacing_lg + 4));
  const bool has_icon = !icon().isNull();
  const bool has_text = !text().isEmpty();
  const QFontMetrics metrics(painter.font());
  int total_width = has_text ? MnemonicTextWidth(metrics, text()) : 0;
  if (has_icon) total_width += icon_extent;
  if (has_icon && has_text) total_width += tokens.spacing_sm;
  int x = content_align_ == ContentAlign::kStart
              ? content.left()
              : content.center().x() - total_width / 2;

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

  // Disclosure / subtle headers already sit inside a Surface — skip rings.
  // Other buttons get a single focus stroke (no outer+inner double halo).
  if (hasFocus() &&
      !(variant_ == Variant::kSubtle && isCheckable())) {
    painter.setBrush(Qt::NoBrush);
    painter.setPen(QPen(tokens.focus_ring, 1.5));
    painter.drawRoundedRect(QRectF(rect()).adjusted(2.0, 2.0, -2.0, -2.0),
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

/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/input/color_picker.hpp"

#include <QColorDialog>
#include <QFontMetrics>
#include <QPaintEvent>
#include <QPainter>

namespace autoviz::widget {

ColorPicker::ColorPicker(QWidget* parent) : QAbstractButton(parent) {
  setObjectName(QStringLiteral("AutovizColorPicker"));
  setProperty("autovizRole", QStringLiteral("colorPicker"));
  setFocusPolicy(Qt::StrongFocus);
  setCursor(Qt::PointingHandCursor);
  connect(this, &QAbstractButton::clicked, this, &ColorPicker::OpenEditor);
  UpdateAccessibility();
}

void ColorPicker::SetColor(const QColor& color) {
  if (!color.isValid() || color_ == color) {
    return;
  }
  color_ = color;
  if (!alpha_enabled_) {
    color_.setAlpha(255);
  }
  UpdateAccessibility();
  update();
  emit ColorChanged(color_);
}

void ColorPicker::SetAlphaEnabled(bool enabled) {
  if (alpha_enabled_ == enabled) {
    return;
  }
  alpha_enabled_ = enabled;
  if (!alpha_enabled_ && color_.alpha() != 255) {
    color_.setAlpha(255);
    emit ColorChanged(color_);
  }
  UpdateAccessibility();
  updateGeometry();
  update();
}

void ColorPicker::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  update();
}

QSize ColorPicker::sizeHint() const {
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  const QString text =
      color_.name(alpha_enabled_ ? QColor::HexArgb : QColor::HexRgb).toUpper();
  return QSize(tokens.spacing_lg * 2 + tokens.control_height_small +
                   fontMetrics().horizontalAdvance(text),
               tokens.control_height_medium);
}

QSize ColorPicker::minimumSizeHint() const {
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  return QSize(tokens.control_height_large + tokens.spacing_lg,
               tokens.control_height_small);
}

void ColorPicker::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  const QColor border = hasFocus() ? tokens.focus_ring : tokens.border;
  const QColor background =
      isDown() ? tokens.surface_hovered : tokens.surface_raised;

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(QPen(border, hasFocus() ? tokens.focus_ring_width : 1));
  painter.setBrush(background);
  painter.drawRoundedRect(QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5),
                          tokens.radius_small, tokens.radius_small);

  const int swatch_size = qMax(12, height() - 2 * tokens.spacing_sm);
  const QRect swatch(tokens.spacing_sm, (height() - swatch_size) / 2,
                     swatch_size, swatch_size);
  painter.setPen(QPen(tokens.border, 1));
  painter.setBrush(isEnabled() ? color_ : tokens.disabled);
  painter.drawRoundedRect(QRectF(swatch).adjusted(0.5, 0.5, -0.5, -0.5),
                          tokens.radius_small / 2.0, tokens.radius_small / 2.0);

  const QRect text_rect = rect().adjusted(swatch.right() + tokens.spacing_sm, 0,
                                          -tokens.spacing_sm, 0);
  painter.setPen(isEnabled() ? tokens.text : tokens.disabled);
  const QString color_text =
      color_.name(alpha_enabled_ ? QColor::HexArgb : QColor::HexRgb).toUpper();
  painter.drawText(
      text_rect, Qt::AlignLeft | Qt::AlignVCenter,
      fontMetrics().elidedText(color_text, Qt::ElideRight, text_rect.width()));
}

void ColorPicker::OpenEditor() {
  QColorDialog::ColorDialogOptions options;
  if (alpha_enabled_) {
    options |= QColorDialog::ShowAlphaChannel;
  }
  const QColor selected =
      QColorDialog::getColor(color_, this, tr("Select color"), options);
  if (selected.isValid()) {
    SetColor(selected);
  }
}

void ColorPicker::UpdateAccessibility() {
  if (accessibleName().isEmpty()) {
    setAccessibleName(tr("Color picker"));
  }
  const QString value =
      color_.name(alpha_enabled_ ? QColor::HexArgb : QColor::HexRgb).toUpper();
  setAccessibleDescription(tr("Selected color %1").arg(value));
  setToolTip(value);
}

}  // namespace autoviz::widget

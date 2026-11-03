/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/input/search_box.hpp"

#include <QKeyEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPalette>

namespace autoviz::widget {

SearchBox::SearchBox(QWidget* parent) : QLineEdit(parent) {
  setObjectName(QStringLiteral("AutovizSearchBox"));
  setProperty("autovizRole", QStringLiteral("searchBox"));
  setAccessibleName(tr("Search"));
  setPlaceholderText(tr("Search"));
  setClearButtonEnabled(true);
  setFrame(false);
  setFocusPolicy(Qt::StrongFocus);
  setTextMargins(10, 0, 10, 0);
  QPalette field_palette = palette();
  field_palette.setBrush(QPalette::Base, Qt::transparent);
  setPalette(field_palette);
  setAttribute(Qt::WA_TranslucentBackground);
  connect(this, &QLineEdit::returnPressed, this,
          [this]() { emit SearchRequested(text().trimmed()); });
}

void SearchBox::SetSearchBoxSize(Size size) {
  if (size_ == size) {
    return;
  }
  size_ = size;
  updateGeometry();
  update();
}

void SearchBox::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  update();
}

int SearchBox::ControlHeight(const ThemeTokens& tokens) const noexcept {
  switch (size_) {
    case Size::kCompact:
      return tokens.control_height_compact;
    case Size::kSmall:
      return tokens.control_height_small;
    case Size::kLarge:
      return tokens.control_height_large;
    case Size::kMedium:
      return tokens.control_height_medium;
  }
  return tokens.control_height_medium;
}

QSize SearchBox::sizeHint() const {
  QSize result = QLineEdit::sizeHint();
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  result.setHeight(qMax(ControlHeight(tokens), fontMetrics().height() + 2));
  result.setWidth(qMax(result.width(), 200));
  return result;
}

QSize SearchBox::minimumSizeHint() const {
  QSize result = QLineEdit::minimumSizeHint();
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  result.setHeight(qMax(ControlHeight(tokens), fontMetrics().height() + 2));
  return result;
}

void SearchBox::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_Escape && !text().isEmpty()) {
    clear();
    event->accept();
    return;
  }
  QLineEdit::keyPressEvent(event);
}

void SearchBox::paintEvent(QPaintEvent* event) {
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setBrush(isEnabled() ? tokens.surface_raised : tokens.surface);
  painter.setPen(QPen(hasFocus() ? tokens.focus_ring : tokens.border,
                      hasFocus() ? tokens.focus_ring_width : 1));
  const qreal inset = hasFocus() ? tokens.focus_ring_width * 0.5 : 0.5;
  painter.drawRoundedRect(QRectF(rect()).adjusted(inset, inset, -inset, -inset),
                          tokens.radius_small, tokens.radius_small);
  painter.end();
  QLineEdit::paintEvent(event);
}

}  // namespace autoviz::widget

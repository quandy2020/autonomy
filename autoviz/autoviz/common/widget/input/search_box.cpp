/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/input/search_box.hpp"

#include <QEvent>
#include <QKeyEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPalette>

#include "autoviz/common/widget/foundation/validation_state.hpp"

namespace autoviz::widget {
namespace {

void EnsureTransparentEditorBase(QLineEdit* editor) {
  QPalette palette = editor->palette();
  bool changed = false;
  for (const QPalette::ColorGroup group :
       {QPalette::Active, QPalette::Inactive, QPalette::Disabled}) {
    if (palette.brush(group, QPalette::Base).color().alpha() != 0) {
      palette.setBrush(group, QPalette::Base, Qt::transparent);
      changed = true;
    }
  }
  if (changed) {
    editor->setPalette(palette);
  }
}

}  // namespace

SearchBox::SearchBox(QWidget* parent) : QLineEdit(parent) {
  setObjectName(QStringLiteral("AutovizSearchBox"));
  setProperty("autovizRole", QStringLiteral("searchBox"));
  setAccessibleName(tr("Search"));
  setPlaceholderText(tr("Search"));
  setClearButtonEnabled(true);
  setFrame(false);
  setFocusPolicy(Qt::StrongFocus);
  setAttribute(Qt::WA_Hover);
  setTextMargins(10, 0, 10, 0);
  setAttribute(Qt::WA_TranslucentBackground);
  EnsureTransparentEditorBase(this);
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
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  result.setHeight(qMax(ControlHeight(tokens), fontMetrics().height() + 2));
  result.setWidth(qMax(result.width(), 200));
  return result;
}

QSize SearchBox::minimumSizeHint() const {
  QSize result = QLineEdit::minimumSizeHint();
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  result.setHeight(qMax(ControlHeight(tokens), fontMetrics().height() + 2));
  return result;
}

bool SearchBox::event(QEvent* event) {
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
      // ApplyWidgetTheme() deliberately assigns a complete native palette.
      // Keep only the editor base transparent so native text, selection, IME,
      // cursor, and inline actions remain intact above our rounded material.
      EnsureTransparentEditorBase(this);
      update();
      break;
    default:
      break;
  }
  return QLineEdit::event(event);
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
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  QColor fill = tokens.surface_sunken;
  QColor border = tokens.border_subtle;
  qreal border_width = 1.0;
  if (!isEnabled()) {
    fill = tokens.surface_sunken;
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

  QLineEdit::paintEvent(event);

  // Native text must be painted before the semantic outline. Otherwise a
  // recursively applied Qt palette can cover the focus/error ring with the
  // line edit's rectangular Base primitive.
  QPainter outline(this);
  outline.setRenderHint(QPainter::Antialiasing);
  outline.setBrush(Qt::NoBrush);
  outline.setPen(
      QPen(border, border_width, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  const qreal inset = border_width * 0.5;
  outline.drawRoundedRect(QRectF(rect()).adjusted(inset, inset, -inset, -inset),
                          tokens.radius_small, tokens.radius_small);
}

}  // namespace autoviz::widget

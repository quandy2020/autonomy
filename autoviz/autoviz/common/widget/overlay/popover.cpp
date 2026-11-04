/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/overlay/popover.hpp"

#include <QGuiApplication>
#include <QKeyEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QScreen>
#include <QVBoxLayout>

#include "autoviz/common/widget/foundation/widget_theme.hpp"

namespace autoviz::widget {

Popover::Popover(QWidget* parent)
    : QFrame(parent, Qt::Popup | Qt::FramelessWindowHint),
      layout_(new QVBoxLayout(this)) {
  setObjectName(QStringLiteral("AutovizPopover"));
  setProperty("autovizRole", QStringLiteral("popover"));
  setAccessibleName(tr("Popover"));
  setAttribute(Qt::WA_TranslucentBackground);
  setAutoFillBackground(false);
  setFrameStyle(QFrame::NoFrame);
  setFocusPolicy(Qt::StrongFocus);
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  layout_->setContentsMargins(tokens.spacing_md, tokens.spacing_md,
                              tokens.spacing_md + 2, tokens.spacing_md + 2);
  ApplyWidgetTheme(this, tone_, false);
}

void Popover::SetContentWidget(QWidget* content) {
  if (content_ == content) {
    return;
  }
  if (content_ != nullptr) {
    layout_->removeWidget(content_);
    content_->hide();
    content_->deleteLater();
  }
  content_ = content;
  if (content_ != nullptr) {
    content_->setParent(this);
    layout_->addWidget(content_);
    ApplyWidgetTheme(content_, tone_, true);
  }
  adjustSize();
}

QWidget* Popover::TakeContentWidget() {
  QWidget* content = content_;
  if (content == nullptr) {
    return nullptr;
  }
  layout_->removeWidget(content);
  content->hide();
  content->setParent(nullptr);
  content_ = nullptr;
  adjustSize();
  return content;
}

void Popover::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  layout_->setContentsMargins(tokens.spacing_md, tokens.spacing_md,
                              tokens.spacing_md + 2, tokens.spacing_md + 2);
  ApplyWidgetTheme(this, tone_, true);
  updateGeometry();
  update();
}

void Popover::ShowFor(QWidget* anchor, Placement placement) {
  if (anchor == nullptr) {
    return;
  }
  adjustSize();
  const QRect anchor_rect(anchor->mapToGlobal(QPoint(0, 0)), anchor->size());
  move(PositionFor(anchor_rect, placement));
  show();
  raise();
  setFocus(Qt::PopupFocusReason);
}

void Popover::Dismiss() { close(); }

void Popover::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_Escape) {
    Dismiss();
    event->accept();
    return;
  }
  QFrame::keyPressEvent(event);
}

void Popover::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  QColor shadow = tokens.shadow_ambient;
  shadow.setAlpha(qRound(shadow.alpha() * 0.65));
  painter.setPen(Qt::NoPen);
  painter.setBrush(shadow);
  painter.drawRoundedRect(QRectF(rect()).adjusted(1.5, 2.5, -1.5, -0.5),
                          tokens.radius_large, tokens.radius_large);

  painter.setBrush(tokens.surface_raised);
  painter.setPen(QPen(tokens.border, 1));
  painter.drawRoundedRect(QRectF(rect()).adjusted(0.5, 0.5, -0.5, -2.5),
                          tokens.radius_large, tokens.radius_large);
}

QPoint Popover::PositionFor(const QRect& anchor, Placement placement) const {
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  const QSize popup_size = size();
  int x = anchor.left();
  int y = anchor.bottom() + tokens.spacing_xs;
  switch (placement) {
    case Placement::kBottomEnd:
      x = anchor.right() - popup_size.width() + 1;
      break;
    case Placement::kTopStart:
      y = anchor.top() - popup_size.height() - tokens.spacing_xs;
      break;
    case Placement::kTopEnd:
      x = anchor.right() - popup_size.width() + 1;
      y = anchor.top() - popup_size.height() - tokens.spacing_xs;
      break;
    case Placement::kBottomStart:
      break;
  }

  QScreen* screen = QGuiApplication::screenAt(anchor.center());
  if (screen == nullptr) {
    screen = QGuiApplication::primaryScreen();
  }
  if (screen == nullptr) {
    return QPoint(x, y);
  }
  const QRect available = screen->availableGeometry();
  x = qBound(
      available.left(), x,
      qMax(available.left(), available.right() - popup_size.width() + 1));
  y = qBound(
      available.top(), y,
      qMax(available.top(), available.bottom() - popup_size.height() + 1));
  return QPoint(x, y);
}

}  // namespace autoviz::widget

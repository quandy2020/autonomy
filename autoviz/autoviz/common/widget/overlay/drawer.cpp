/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/overlay/drawer.hpp"

#include <QAbstractAnimation>
#include <QEasingCurve>
#include <QEvent>
#include <QHideEvent>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPropertyAnimation>
#include <QShowEvent>
#include <QStyle>
#include <QVariant>
#include <algorithm>

namespace autoviz::widget {

Drawer::Drawer(QWidget* parent)
    : QWidget(parent),
      animation_(new QPropertyAnimation(this, "progress", this)),
      observed_parent_(parent) {
  setObjectName(QStringLiteral("AutovizDrawer"));
  setProperty("autovizRole", QStringLiteral("drawer"));
  setAccessibleName(tr("Drawer"));
  setFocusPolicy(Qt::StrongFocus);
  setAttribute(Qt::WA_StyledBackground, false);
  setAttribute(Qt::WA_OpaquePaintEvent, false);
  hide();

  animation_->setEasingCurve(QEasingCurve::OutCubic);
  connect(animation_, &QPropertyAnimation::finished, this, [this]() {
    if (closing_) {
      closing_ = false;
      hide();
      emit Closed();
      return;
    }
    if (content_ != nullptr) {
      content_->setFocus(Qt::OtherFocusReason);
    }
    emit Opened();
  });

  if (parent != nullptr) {
    parent->installEventFilter(this);
    SyncToParent();
  }
}

Drawer::~Drawer() {
  if (observed_parent_ != nullptr) {
    observed_parent_->removeEventFilter(this);
  }
}

void Drawer::SetPlacement(Placement placement) {
  if (placement_ == placement) {
    return;
  }
  placement_ = placement;
  UpdateContentGeometry();
  update();
}

void Drawer::SetPanelExtent(int extent) {
  const int bounded = qBound(160, extent, 1600);
  if (panel_extent_ == bounded) {
    return;
  }
  panel_extent_ = bounded;
  UpdateContentGeometry();
  update();
}

void Drawer::SetDismissOnScrimClick(bool enabled) {
  dismiss_on_scrim_click_ = enabled;
}

void Drawer::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  update();
}

void Drawer::SetContentWidget(QWidget* content) {
  if (content_ == content) {
    return;
  }
  if (content_ != nullptr) {
    content_->hide();
    content_->deleteLater();
  }
  content_ = content;
  if (content_ != nullptr) {
    content_->setParent(this);
    content_->show();
  }
  UpdateContentGeometry();
}

QWidget* Drawer::TakeContentWidget() {
  QWidget* content = content_;
  if (content == nullptr) {
    return nullptr;
  }
  content->hide();
  content->setParent(nullptr);
  content_ = nullptr;
  return content;
}

void Drawer::Show() {
  if (parentWidget() == nullptr) {
    return;
  }
  if (isVisible() && !closing_ && qFuzzyCompare(progress_ + 1.0, 2.0)) {
    return;
  }
  const bool was_visible = isVisible();
  closing_ = false;
  animation_->stop();
  SyncToParent();
  show();
  raise();
  setFocus(Qt::PopupFocusReason);

  if (!AnimationsEnabled()) {
    SetProgress(1.0);
    if (content_ != nullptr) {
      content_->setFocus(Qt::OtherFocusReason);
    }
    if (!was_visible) {
      emit Opened();
    }
    return;
  }
  if (!was_visible) {
    SetProgress(0.0);
  }
  animation_->setDuration(GetThemeTokens(tone_).animation_normal_ms);
  animation_->setStartValue(progress_);
  animation_->setEndValue(1.0);
  animation_->start(QAbstractAnimation::KeepWhenStopped);
}

void Drawer::Dismiss() {
  if (!isVisible() || closing_) {
    return;
  }
  closing_ = true;
  animation_->stop();
  if (!AnimationsEnabled()) {
    SetProgress(0.0);
    closing_ = false;
    hide();
    emit Closed();
    return;
  }
  animation_->setDuration(GetThemeTokens(tone_).animation_normal_ms);
  animation_->setStartValue(progress_);
  animation_->setEndValue(0.0);
  animation_->start(QAbstractAnimation::KeepWhenStopped);
}

bool Drawer::event(QEvent* event) {
  if (event->type() == QEvent::ParentAboutToChange &&
      observed_parent_ != nullptr) {
    observed_parent_->removeEventFilter(this);
    observed_parent_.clear();
  }
  const bool handled = QWidget::event(event);
  if (event->type() == QEvent::ParentChange) {
    observed_parent_ = parentWidget();
    if (observed_parent_ != nullptr) {
      observed_parent_->installEventFilter(this);
      SyncToParent();
    }
  }
  return handled;
}

bool Drawer::eventFilter(QObject* watched, QEvent* event) {
  if (watched == observed_parent_) {
    if (event->type() == QEvent::Resize) {
      SyncToParent();
    } else if (event->type() == QEvent::Hide) {
      animation_->stop();
      closing_ = false;
      SetProgress(0.0);
      hide();
    }
  }
  return QWidget::eventFilter(watched, event);
}

void Drawer::hideEvent(QHideEvent* event) {
  animation_->stop();
  QWidget::hideEvent(event);
}

void Drawer::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_Escape) {
    Dismiss();
    event->accept();
    return;
  }
  QWidget::keyPressEvent(event);
}

void Drawer::mousePressEvent(QMouseEvent* event) {
  if (dismiss_on_scrim_click_ && !PanelRect().contains(event->position())) {
    Dismiss();
    event->accept();
    return;
  }
  QWidget::mousePressEvent(event);
}

void Drawer::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  QColor scrim(Qt::black);
  scrim.setAlpha(qRound(88.0 * progress_));
  painter.fillRect(rect(), scrim);

  painter.setPen(QPen(tokens.border, 1));
  painter.setBrush(tokens.surface_raised);
  painter.drawRect(PanelRect());
}

void Drawer::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  SyncToParent();
}

void Drawer::SetProgress(qreal progress) {
  const qreal bounded = std::clamp(progress, 0.0, 1.0);
  if (qFuzzyCompare(progress_ + 1.0, bounded + 1.0)) {
    return;
  }
  progress_ = bounded;
  UpdateContentGeometry();
  update();
}

QRectF Drawer::PanelRect() const {
  const qreal extent =
      (placement_ == Placement::kLeft || placement_ == Placement::kRight)
          ? std::min(panel_extent_, width())
          : std::min(panel_extent_, height());
  switch (placement_) {
    case Placement::kLeft:
      return QRectF(-extent + extent * progress_, 0, extent, height());
    case Placement::kRight:
      return QRectF(width() - extent * progress_, 0, extent, height());
    case Placement::kTop:
      return QRectF(0, -extent + extent * progress_, width(), extent);
    case Placement::kBottom:
      return QRectF(0, height() - extent * progress_, width(), extent);
  }
  return {};
}

void Drawer::SyncToParent() {
  if (QWidget* host = parentWidget(); host != nullptr) {
    setGeometry(host->rect());
  }
  UpdateContentGeometry();
}

void Drawer::UpdateContentGeometry() {
  if (content_ == nullptr) {
    return;
  }
  const int margin = GetThemeTokens(tone_).spacing_lg;
  content_->setGeometry(
      PanelRect().toAlignedRect().adjusted(margin, margin, -margin, -margin));
}

bool Drawer::AnimationsEnabled() const {
  return isVisible() && isEnabled() &&
         style()->styleHint(QStyle::SH_Widget_Animate, nullptr, this) != 0;
}

}  // namespace autoviz::widget

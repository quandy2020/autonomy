/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/overlay/drawer.hpp"

#include <QAbstractAnimation>
#include <QApplication>
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

#include "autoviz/common/widget/foundation/motion_policy.hpp"
#include "autoviz/common/widget/foundation/widget_theme.hpp"

namespace autoviz::widget {
namespace {

QWidget* FindFocusableDescendant(const QWidget* drawer, QWidget* start,
                                 bool next) {
  QWidget* candidate = start;
  do {
    candidate = next ? candidate->nextInFocusChain()
                     : candidate->previousInFocusChain();
    if (candidate != nullptr && candidate != drawer &&
        drawer->isAncestorOf(candidate) && !candidate->isWindow() &&
        candidate->isVisibleTo(drawer) && candidate->isEnabled() &&
        (candidate->focusPolicy() & Qt::TabFocus) != 0) {
      return candidate;
    }
  } while (candidate != nullptr && candidate != start);
  return nullptr;
}

}  // namespace

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
      RestorePreviousFocus();
      emit Closed();
      return;
    }
    FocusFirstDescendant();
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
  if (content_ != nullptr) {
    ApplyWidgetTheme(content_, tone_, true);
  }
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
    ApplyWidgetTheme(content_, tone_, true);
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
  if (!was_visible) {
    previous_focus_ = QApplication::focusWidget();
  }
  closing_ = false;
  animation_->stop();
  SyncToParent();
  show();
  raise();
  setFocus(Qt::PopupFocusReason);

  if (!AnimationsEnabled()) {
    SetProgress(1.0);
    FocusFirstDescendant();
    if (!was_visible) {
      emit Opened();
    }
    return;
  }
  if (!was_visible) {
    SetProgress(0.0);
  }
  animation_->setDuration(MotionPolicy::ResolveDuration(
      GetThemeTokens(this, tone_).animation_normal_ms));
  animation_->setEasingCurve(QEasingCurve::OutCubic);
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
    RestorePreviousFocus();
    emit Closed();
    return;
  }
  animation_->setDuration(MotionPolicy::ResolveDuration(
      GetThemeTokens(this, tone_).animation_normal_ms));
  animation_->setEasingCurve(QEasingCurve::InCubic);
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

bool Drawer::focusNextPrevChild(bool next) {
  if (!isVisible()) {
    return QWidget::focusNextPrevChild(next);
  }
  QWidget* current = QApplication::focusWidget();
  if (current == nullptr || (current != this && !isAncestorOf(current))) {
    current = this;
  }
  QWidget* target = FindFocusableDescendant(this, current, next);
  if (target != nullptr) {
    target->setFocus(next ? Qt::TabFocusReason : Qt::BacktabFocusReason);
  } else {
    setFocus(next ? Qt::TabFocusReason : Qt::BacktabFocusReason);
  }
  return true;
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
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  QColor scrim = tokens.scrim;
  scrim.setAlpha(qRound(scrim.alpha() * progress_));
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

void Drawer::FocusFirstDescendant() {
  QWidget* target = FindFocusableDescendant(this, this, true);
  if (target != nullptr) {
    target->setFocus(Qt::OtherFocusReason);
  } else {
    setFocus(Qt::PopupFocusReason);
  }
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
  const int margin = GetThemeTokens(this, tone_).spacing_lg;
  content_->setGeometry(
      PanelRect().toAlignedRect().adjusted(margin, margin, -margin, -margin));
}

void Drawer::RestorePreviousFocus() {
  if (previous_focus_ != nullptr && previous_focus_->isVisible() &&
      previous_focus_->isEnabled()) {
    previous_focus_->setFocus(Qt::PopupFocusReason);
  }
  previous_focus_.clear();
}

bool Drawer::AnimationsEnabled() const {
  return isVisible() && isEnabled() && !MotionPolicy::ReducedMotionEnabled();
}

}  // namespace autoviz::widget

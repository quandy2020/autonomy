/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/layout/collapsible_panel.hpp"

#include <QAbstractAnimation>
#include <QEasingCurve>
#include <QHideEvent>
#include <QPropertyAnimation>
#include <QSignalBlocker>
#include <QStyle>
#include <QVBoxLayout>
#include <QVariant>

#include "autoviz/common/widget/foundation/motion_policy.hpp"
#include "autoviz/common/widget/input/button.hpp"

namespace autoviz::widget {

CollapsiblePanel::CollapsiblePanel(QWidget* parent)
    : Surface(parent),
      header_button_(new Button(this)),
      content_host_(new QWidget(this)),
      content_layout_(new QVBoxLayout(content_host_)),
      animation_(new QPropertyAnimation(content_host_, "maximumHeight", this)) {
  setObjectName(QStringLiteral("AutovizCollapsiblePanel"));
  setProperty("autovizRole", QStringLiteral("collapsiblePanel"));
  SetElevation(Elevation::kFlat);

  const ThemeTokens& tokens = GetThemeTokens();
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(tokens.spacing_sm, tokens.spacing_sm,
                           tokens.spacing_sm, tokens.spacing_sm);
  root->setSpacing(tokens.spacing_xs);

  header_button_->SetVariant(Button::Variant::kSubtle);
  header_button_->setCheckable(true);
  header_button_->setChecked(true);
  header_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  header_button_->setAccessibleName(tr("Section"));
  connect(header_button_, &QAbstractButton::toggled, this,
          &CollapsiblePanel::SetExpanded);
  root->addWidget(header_button_);

  content_layout_->setContentsMargins(0, 0, 0, 0);
  content_layout_->setSpacing(0);
  root->addWidget(content_host_);

  animation_->setEasingCurve(QEasingCurve::OutCubic);
  animation_->setDuration(
      MotionPolicy::ResolveDuration(tokens.animation_normal_ms));
  connect(animation_, &QPropertyAnimation::finished, this,
          &CollapsiblePanel::FinishTransition);
  UpdateHeader();
}

QString CollapsiblePanel::title() const { return header_button_->text(); }

void CollapsiblePanel::SetTitle(const QString& title) {
  header_button_->setText(title);
  header_button_->setAccessibleName(title);
  updateGeometry();
}

void CollapsiblePanel::SetExpanded(bool expanded) {
  if (expanded_ == expanded) {
    return;
  }
  expanded_ = expanded;
  {
    const QSignalBlocker blocker(header_button_);
    header_button_->setChecked(expanded);
  }
  UpdateHeader();
  StartTransition();
  emit ExpandedChanged(expanded_);
}

void CollapsiblePanel::SetAnimationEnabled(bool enabled) {
  animation_enabled_ = enabled;
  if (!enabled && animation_->state() == QAbstractAnimation::Running) {
    animation_->stop();
    FinishTransition();
  }
}

void CollapsiblePanel::SetTone(style::Tone tone) {
  Surface::SetTone(tone);
  header_button_->SetTone(tone);
}

void CollapsiblePanel::SetContentWidget(QWidget* content) {
  if (content_ == content) {
    return;
  }
  if (content_ != nullptr) {
    content_layout_->removeWidget(content_);
    content_->hide();
    content_->deleteLater();
  }
  content_ = content;
  if (content_ != nullptr) {
    content_->setParent(content_host_);
    content_layout_->addWidget(content_);
    // Keep the child logically shown while the host clips it. This preserves
    // a valid layout sizeHint when expanding from the collapsed state.
    content_->show();
  }
  FinishTransition();
  updateGeometry();
}

QWidget* CollapsiblePanel::TakeContentWidget() {
  QWidget* content = content_;
  if (content == nullptr) {
    return nullptr;
  }
  content_layout_->removeWidget(content);
  content->hide();
  content->setParent(nullptr);
  content_ = nullptr;
  FinishTransition();
  updateGeometry();
  return content;
}

void CollapsiblePanel::hideEvent(QHideEvent* event) {
  animation_->stop();
  FinishTransition();
  Surface::hideEvent(event);
}

void CollapsiblePanel::UpdateHeader() {
  header_button_->setIcon(style()->standardIcon(
      expanded_ ? QStyle::SP_ArrowDown : QStyle::SP_ArrowRight));
  header_button_->setAccessibleDescription(expanded_ ? tr("Expanded")
                                                     : tr("Collapsed"));
}

void CollapsiblePanel::StartTransition() {
  animation_->stop();
  const int duration =
      MotionPolicy::ResolveDuration(GetThemeTokens(tone()).animation_normal_ms);
  const bool animations_enabled =
      animation_enabled_ && isVisible() && isEnabled() && duration > 0;
  if (!animations_enabled) {
    FinishTransition();
    return;
  }

  content_host_->show();
  animation_->setDuration(duration);
  animation_->setStartValue(content_host_->height());
  animation_->setEndValue(expanded_ ? ExpandedHeight() : 0);
  animation_->start(QAbstractAnimation::KeepWhenStopped);
}

void CollapsiblePanel::FinishTransition() {
  if (expanded_) {
    content_host_->setMaximumHeight(QWIDGETSIZE_MAX);
    content_host_->show();
    if (content_ != nullptr) {
      content_->show();
    }
  } else {
    content_host_->setMaximumHeight(0);
    content_host_->hide();
  }
  updateGeometry();
}

int CollapsiblePanel::ExpandedHeight() const {
  return qMax(0, content_layout_->sizeHint().height());
}

}  // namespace autoviz::widget

/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/layout/collapsible_panel.hpp"

#include <QAbstractAnimation>
#include <QEasingCurve>
#include <QEvent>
#include <QHideEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPropertyAnimation>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <QVariant>

#include "autoviz/common/widget/foundation/motion_policy.hpp"
#include "autoviz/common/widget/foundation/semantic_icon.hpp"
#include "autoviz/common/widget/foundation/theme_tokens.hpp"
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

  const ThemeTokens& tokens = GetThemeTokens(this, tone());
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(tokens.spacing_sm, tokens.spacing_sm,
                           tokens.spacing_sm, tokens.spacing_sm);
  root->setSpacing(tokens.spacing_xs);

  header_button_->SetVariant(Button::Variant::kSubtle);
  header_button_->SetButtonSize(Button::Size::kSmall);
  header_button_->SetContentAlign(Button::ContentAlign::kStart);
  header_button_->SetTone(tone());
  header_button_->setCheckable(true);
  header_button_->setChecked(true);
  header_button_->setFocusPolicy(Qt::ClickFocus);
  header_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  header_button_->setAccessibleName(tr("Section"));
  QFont header_font = header_button_->font();
  header_font.setWeight(QFont::DemiBold);
  header_font.setPointSizeF(qMax(10.0, header_font.pointSizeF()));
  header_button_->setFont(header_font);
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
  UpdateHeader();
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
    // Keep the child logically shown while the host clips it. This
    // preserves a valid layout sizeHint when expanding from the collapsed
    // state.
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

void CollapsiblePanel::changeEvent(QEvent* event) {
  Surface::changeEvent(event);
  switch (event->type()) {
    case QEvent::EnabledChange:
    case QEvent::PaletteChange:
    case QEvent::ScreenChangeInternal:
    case QEvent::StyleChange:
      UpdateHeader();
      break;
    default:
      break;
  }
}

void CollapsiblePanel::hideEvent(QHideEvent* event) {
  animation_->stop();
  FinishTransition();
  Surface::hideEvent(event);
}

void CollapsiblePanel::paintEvent(QPaintEvent* /*event*/) {
  // One flat plate + one rim — avoids Surface Liquid wash / focus double halo
  // around Series and other settings section headers.
  const ThemeTokens& tokens = GetThemeTokens(this, tone());
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  QColor fill = tokens.surface;
  fill.setAlpha(255);
  painter.setBrush(fill);
  painter.setPen(QPen(tokens.border_subtle, 1.0));
  painter.drawRoundedRect(QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5),
                          tokens.radius_medium, tokens.radius_medium);
}

void CollapsiblePanel::UpdateHeader() {
  const ThemeTokens& tokens = GetThemeTokens(this, tone());
  const QColor ink = expanded_ ? tokens.accent_text : tokens.text_muted;
  QPalette pal = header_button_->palette();
  pal.setColor(QPalette::ButtonText, ink);
  pal.setColor(QPalette::Button, Qt::transparent);
  header_button_->setPalette(pal);
  header_button_->setIcon(
      internal::MakePaletteChevronIcon(header_button_->palette(), expanded_));
  header_button_->setAccessibleDescription(expanded_ ? tr("Expanded")
                                                     : tr("Collapsed"));
  header_button_->update();
}

void CollapsiblePanel::StartTransition() {
  animation_->stop();
  const int duration = MotionPolicy::ResolveDuration(
      GetThemeTokens(this, tone()).animation_normal_ms);
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

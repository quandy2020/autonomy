/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/feedback/skeleton.hpp"

#include <QEvent>
#include <QGuiApplication>
#include <QHideEvent>
#include <QLinearGradient>
#include <QPaintEvent>
#include <QPainter>
#include <QShowEvent>
#include <QStyle>
#include <QTimerEvent>

#include "autoviz/common/widget/foundation/motion_policy.hpp"

namespace autoviz::widget {

Skeleton::Skeleton(QWidget* parent) : QWidget(parent) {
  setObjectName(QStringLiteral("AutovizSkeleton"));
  setProperty("autovizRole", QStringLiteral("skeleton"));
  setAccessibleName(tr("Loading"));
  setAttribute(Qt::WA_TransparentForMouseEvents);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  if (qGuiApp != nullptr) {
    connect(qGuiApp, &QGuiApplication::applicationStateChanged, this,
            [this](Qt::ApplicationState) { SyncAnimation(); });
  }
}

void Skeleton::SetActive(bool active) {
  if (active_ == active) {
    return;
  }
  active_ = active;
  SyncAnimation();
  update();
}

void Skeleton::SetCornerRadius(int radius) {
  const int bounded = qBound(0, radius, 64);
  if (corner_radius_ == bounded) {
    return;
  }
  corner_radius_ = bounded;
  update();
}

void Skeleton::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  update();
}

QSize Skeleton::sizeHint() const { return QSize(160, 16); }

void Skeleton::changeEvent(QEvent* event) {
  QWidget::changeEvent(event);
  if (event->type() == QEvent::EnabledChange ||
      event->type() == QEvent::StyleChange) {
    SyncAnimation();
  }
}

void Skeleton::hideEvent(QHideEvent* event) {
  QWidget::hideEvent(event);
  SyncAnimation();
}

void Skeleton::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  QColor base = tokens.border;
  base.setAlpha(isEnabled() ? 105 : 60);
  QColor highlight = tokens.surface_raised;
  highlight.setAlpha(isEnabled() ? 220 : 110);

  const qreal sweep = qMax(1, width());
  const qreal center = (phase_ / 100.0) * 1.8 - 0.4;
  QLinearGradient gradient(0, 0, sweep, 0);
  gradient.setColorAt(0.0, base);
  gradient.setColorAt(qBound(0.0, center - 0.18, 1.0), base);
  gradient.setColorAt(qBound(0.0, center, 1.0), highlight);
  gradient.setColorAt(qBound(0.0, center + 0.18, 1.0), base);
  gradient.setColorAt(1.0, base);

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(Qt::NoPen);
  painter.setBrush(gradient);
  painter.drawRoundedRect(rect(), corner_radius_, corner_radius_);
}

void Skeleton::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  SyncAnimation();
}

void Skeleton::timerEvent(QTimerEvent* event) {
  if (event->timerId() != timer_.timerId()) {
    QWidget::timerEvent(event);
    return;
  }
  phase_ = (phase_ + 3) % 100;
  update();
}

void Skeleton::SyncAnimation() {
  const bool should_run =
      active_ && isVisible() && isEnabled() &&
      QGuiApplication::applicationState() == Qt::ApplicationActive &&
      !MotionPolicy::ReducedMotionEnabled();
  if (should_run && !timer_.isActive()) {
    timer_.start(30, Qt::CoarseTimer, this);
  } else if (!should_run && timer_.isActive()) {
    timer_.stop();
  }
}

}  // namespace autoviz::widget

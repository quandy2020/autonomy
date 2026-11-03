/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/navigation/step_indicator.hpp"

#include <QFontMetrics>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPointer>
#include <QResizeEvent>
#include <QVariant>
#include <utility>

namespace autoviz::widget {
namespace {

constexpr qreal kCircleRadius = 10.0;
constexpr qreal kCircleCenterY = 16.0;

}  // namespace

StepIndicator::StepIndicator(QWidget* parent) : QWidget(parent) {
  setObjectName(QStringLiteral("AutovizStepIndicator"));
  setProperty("autovizRole", QStringLiteral("stepIndicator"));
  setFocusPolicy(Qt::NoFocus);
  setAccessibleName(tr("Progress steps"));
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
}

void StepIndicator::SetSteps(QVector<StepItem> steps) {
  const int previous_index = current_index_;
  steps_ = std::move(steps);
  if (steps_.isEmpty()) {
    current_index_ = -1;
  } else if (!IsSelectable(current_index_)) {
    current_index_ = -1;
    for (int index = 0; index < static_cast<int>(steps_.size()); ++index) {
      if (IsSelectable(index)) {
        current_index_ = index;
        break;
      }
    }
  }
  RebuildGeometry();
  UpdateAccessibility();
  updateGeometry();
  update();
  if (previous_index != current_index_) {
    emit CurrentIndexChanged(current_index_);
  }
}

void StepIndicator::SetCurrentIndex(int index) {
  if (!IsSelectable(index) || current_index_ == index) {
    return;
  }
  current_index_ = index;
  UpdateAccessibility();
  update();
  emit CurrentIndexChanged(index);
}

void StepIndicator::SetInteractive(bool interactive) {
  if (interactive_ == interactive) {
    return;
  }
  interactive_ = interactive;
  setFocusPolicy(interactive ? Qt::StrongFocus : Qt::NoFocus);
  setCursor(interactive ? Qt::PointingHandCursor : Qt::ArrowCursor);
}

void StepIndicator::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  update();
}

QSize StepIndicator::sizeHint() const {
  return QSize(qMax(240, static_cast<int>(steps_.size()) * 112), 56);
}

QSize StepIndicator::minimumSizeHint() const {
  return QSize(qMax(120, static_cast<int>(steps_.size()) * 48), 48);
}

void StepIndicator::keyPressEvent(QKeyEvent* event) {
  if (!interactive_) {
    QWidget::keyPressEvent(event);
    return;
  }
  switch (event->key()) {
    case Qt::Key_Left:
      MoveCurrent(-1);
      event->accept();
      return;
    case Qt::Key_Right:
      MoveCurrent(1);
      event->accept();
      return;
    case Qt::Key_Home:
      for (int index = 0; index < static_cast<int>(steps_.size()); ++index) {
        if (IsSelectable(index)) {
          SetCurrentIndex(index);
          break;
        }
      }
      event->accept();
      return;
    case Qt::Key_End:
      for (int index = static_cast<int>(steps_.size()) - 1; index >= 0;
           --index) {
        if (IsSelectable(index)) {
          SetCurrentIndex(index);
          break;
        }
      }
      event->accept();
      return;
    case Qt::Key_Return:
    case Qt::Key_Enter:
    case Qt::Key_Space:
      Activate(current_index_);
      event->accept();
      return;
    default:
      QWidget::keyPressEvent(event);
  }
}

void StepIndicator::mouseReleaseEvent(QMouseEvent* event) {
  if (interactive_ && event->button() == Qt::LeftButton) {
    const int index = HitTest(event->position());
    if (index >= 0) {
      setFocus(Qt::MouseFocusReason);
      Activate(index);
      event->accept();
      return;
    }
  }
  QWidget::mouseReleaseEvent(event);
}

void StepIndicator::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  if (steps_.isEmpty() || centers_.size() != steps_.size()) {
    return;
  }

  const ThemeTokens& tokens = GetThemeTokens(tone_);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  if (centers_.size() > 1) {
    painter.setPen(QPen(tokens.border, 2));
    painter.drawLine(centers_.front(), centers_.back());
    if (current_index_ > 0) {
      painter.setPen(QPen(tokens.accent, 2));
      painter.drawLine(centers_.front(), centers_[current_index_]);
    }
  }

  const int cell_width = width() / static_cast<int>(steps_.size());
  for (int index = 0; index < static_cast<int>(steps_.size()); ++index) {
    const bool current = index == current_index_;
    const bool complete = index < current_index_;
    const QColor accent =
        steps_[index].enabled ? tokens.accent : tokens.disabled;
    painter.setPen(QPen(current || complete ? accent : tokens.border,
                        current ? tokens.focus_ring_width : 1));
    painter.setBrush(complete ? accent : tokens.surface_raised);
    painter.drawEllipse(centers_[index], kCircleRadius, kCircleRadius);

    if (current && interactive_ && hasFocus()) {
      painter.setBrush(Qt::NoBrush);
      painter.setPen(QPen(tokens.focus_ring, tokens.focus_ring_width));
      painter.drawEllipse(centers_[index], kCircleRadius + 4,
                          kCircleRadius + 4);
    }

    if (complete) {
      painter.setPen(tokens.on_accent);
      painter.drawText(QRectF(centers_[index].x() - kCircleRadius,
                              centers_[index].y() - kCircleRadius,
                              kCircleRadius * 2.0, kCircleRadius * 2.0),
                       Qt::AlignCenter, QStringLiteral("✓"));
    }

    painter.setPen(steps_[index].enabled
                       ? (current ? tokens.accent : tokens.text)
                       : tokens.disabled);
    const QRect text_rect(index * cell_width, 31, cell_width, height() - 31);
    const QString title = fontMetrics().elidedText(
        steps_[index].title, Qt::ElideRight, qMax(8, cell_width - 8));
    painter.drawText(text_rect, Qt::AlignHCenter | Qt::AlignTop, title);
  }
}

void StepIndicator::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  RebuildGeometry();
}

void StepIndicator::Activate(int index) {
  if (!IsSelectable(index)) {
    return;
  }
  QPointer<StepIndicator> guard(this);
  SetCurrentIndex(index);
  if (guard == nullptr) {
    return;
  }
  emit StepActivated(index);
}

void StepIndicator::MoveCurrent(int direction) {
  if (steps_.isEmpty()) {
    return;
  }
  int index = current_index_;
  if (index < 0) {
    index = direction > 0 ? -1 : static_cast<int>(steps_.size());
  }
  for (int candidate = index + direction;
       candidate >= 0 && candidate < static_cast<int>(steps_.size());
       candidate += direction) {
    if (IsSelectable(candidate)) {
      SetCurrentIndex(candidate);
      return;
    }
  }
}

void StepIndicator::RebuildGeometry() {
  centers_.clear();
  const int count = static_cast<int>(steps_.size());
  if (count <= 0) {
    return;
  }
  centers_.reserve(count);
  const qreal cell_width = static_cast<qreal>(width()) / count;
  for (int index = 0; index < count; ++index) {
    centers_.push_back(QPointF((index + 0.5) * cell_width, kCircleCenterY));
  }
}

int StepIndicator::HitTest(const QPointF& position) const {
  if (steps_.isEmpty() || !rect().contains(position.toPoint())) {
    return -1;
  }
  const int count = static_cast<int>(steps_.size());
  const int index = qBound(
      0, static_cast<int>(position.x() * count / qMax(1, width())), count - 1);
  return IsSelectable(index) ? index : -1;
}

bool StepIndicator::IsSelectable(int index) const {
  return index >= 0 && index < static_cast<int>(steps_.size()) &&
         steps_[index].enabled;
}

void StepIndicator::UpdateAccessibility() {
  if (current_index_ < 0 || steps_.isEmpty()) {
    setAccessibleDescription(tr("No steps"));
    return;
  }
  setAccessibleDescription(tr("Step %1 of %2: %3")
                               .arg(current_index_ + 1)
                               .arg(steps_.size())
                               .arg(steps_[current_index_].title));
}

}  // namespace autoviz::widget

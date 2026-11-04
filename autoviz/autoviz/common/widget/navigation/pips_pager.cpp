/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/navigation/pips_pager.hpp"

#include <QEvent>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QResizeEvent>
#include <limits>

#include "autoviz/common/widget/navigation/navigation_direction.hpp"

namespace autoviz::widget {
namespace {

constexpr qreal kPipDiameter = 8.0;
constexpr qreal kCurrentPipDiameter = 12.0;
constexpr qreal kPipSpacing = 10.0;

}  // namespace

PipsPager::PipsPager(QWidget* parent) : QWidget(parent) {
  setObjectName(QStringLiteral("AutovizPipsPager"));
  setProperty("autovizRole", QStringLiteral("pipsPager"));
  setAccessibleName(tr("Page selector"));
  setFocusPolicy(Qt::StrongFocus);
  setCursor(Qt::PointingHandCursor);
  setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  UpdateAccessibility();
}

void PipsPager::SetPageCount(int count) {
  const int bounded = qMax(0, count);
  if (page_count_ == bounded) {
    return;
  }
  const int previous_page = current_page_;
  page_count_ = bounded;
  if (page_count_ == 0) {
    current_page_ = -1;
  } else {
    current_page_ = qBound(0, current_page_, page_count_ - 1);
  }
  RebuildGeometry();
  UpdateAccessibility();
  updateGeometry();
  update();
  if (current_page_ != previous_page) {
    emit CurrentPageChanged(current_page_);
  }
}

void PipsPager::SetCurrentPage(int page) {
  const int bounded = page_count_ > 0 ? qBound(0, page, page_count_ - 1) : -1;
  if (current_page_ == bounded) {
    return;
  }
  current_page_ = bounded;
  RebuildGeometry();
  UpdateAccessibility();
  update();
  emit CurrentPageChanged(current_page_);
}

void PipsPager::SetMaximumVisiblePips(int count) {
  const int bounded = qBound(1, count, 31);
  if (maximum_visible_pips_ == bounded) {
    return;
  }
  maximum_visible_pips_ = bounded;
  RebuildGeometry();
  updateGeometry();
  update();
}

void PipsPager::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  update();
}

QSize PipsPager::sizeHint() const {
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  const int count = qMin(page_count_, maximum_visible_pips_);
  const int width =
      count > 0
          ? qRound(count * kCurrentPipDiameter + (count - 1) * kPipSpacing)
          : qRound(kCurrentPipDiameter);
  return QSize(qMax(width + 8, tokens.control_height_medium),
               tokens.control_height_medium);
}

QSize PipsPager::minimumSizeHint() const {
  const int touch_extent = GetThemeTokens(this, tone_).control_height_medium;
  return QSize(touch_extent, touch_extent);
}

void PipsPager::changeEvent(QEvent* event) {
  QWidget::changeEvent(event);
  if (event->type() == QEvent::LayoutDirectionChange) {
    RebuildGeometry();
    update();
  }
}

void PipsPager::keyPressEvent(QKeyEvent* event) {
  int next_page = current_page_;
  switch (event->key()) {
    case Qt::Key_Left:
    case Qt::Key_Right:
      next_page += internal::HorizontalKeyStep(event->key(), layoutDirection());
      break;
    case Qt::Key_Up:
      --next_page;
      break;
    case Qt::Key_Down:
      ++next_page;
      break;
    case Qt::Key_Home:
      next_page = 0;
      break;
    case Qt::Key_End:
      next_page = page_count_ - 1;
      break;
    case Qt::Key_Return:
    case Qt::Key_Enter:
    case Qt::Key_Space:
      ActivatePage(current_page_);
      event->accept();
      return;
    default:
      QWidget::keyPressEvent(event);
      return;
  }
  SetCurrentPage(next_page);
  event->accept();
}

void PipsPager::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    const int page = HitTest(event->position());
    if (page >= 0) {
      ActivatePage(page);
      event->accept();
      return;
    }
  }
  QWidget::mouseReleaseEvent(event);
}

void PipsPager::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(Qt::NoPen);

  for (int i = 0; i < pip_rects_.size(); ++i) {
    const bool current = visible_pages_[i] == current_page_;
    painter.setBrush(isEnabled() ? (current ? tokens.accent : tokens.border)
                                 : tokens.disabled);
    painter.drawEllipse(pip_rects_[i]);
  }

  if (hasFocus()) {
    painter.setBrush(Qt::NoBrush);
    painter.setPen(QPen(tokens.focus_ring, tokens.focus_ring_width));
    painter.drawRoundedRect(QRectF(rect()).adjusted(1.5, 1.5, -1.5, -1.5),
                            tokens.radius_small, tokens.radius_small);
  }
}

void PipsPager::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  RebuildGeometry();
}

void PipsPager::ActivatePage(int page) {
  if (page < 0 || page >= page_count_ || !isEnabled()) {
    return;
  }
  SetCurrentPage(page);
  emit PageActivated(page);
}

void PipsPager::RebuildGeometry() {
  visible_pages_.clear();
  pip_rects_.clear();
  if (page_count_ <= 0) {
    return;
  }

  const int visible_count = qMin(page_count_, maximum_visible_pips_);
  const int half = visible_count / 2;
  const int first =
      qBound(0, current_page_ - half, page_count_ - visible_count);
  const qreal total_width =
      visible_count * kCurrentPipDiameter + (visible_count - 1) * kPipSpacing;
  const qreal start_x = (width() - total_width) * 0.5;
  const qreal center_y = height() * 0.5;
  for (int i = 0; i < visible_count; ++i) {
    const int page = first + i;
    const int visual_index =
        internal::VisualIndex(i, visible_count, layoutDirection());
    const qreal x =
        start_x + visual_index * (kCurrentPipDiameter + kPipSpacing);
    const qreal diameter =
        page == current_page_ ? kCurrentPipDiameter : kPipDiameter;
    visible_pages_.push_back(page);
    pip_rects_.push_back(QRectF(x + (kCurrentPipDiameter - diameter) * 0.5,
                                center_y - diameter * 0.5, diameter, diameter));
  }
}

void PipsPager::UpdateAccessibility() {
  setAccessibleDescription(
      page_count_ > 0
          ? tr("Page %1 of %2").arg(current_page_ + 1).arg(page_count_)
          : tr("No pages"));
}

int PipsPager::HitTest(const QPointF& position) const {
  if (pip_rects_.isEmpty() || !QRectF(rect()).contains(position)) {
    return -1;
  }

  // Pip centers stay fixed when the selected pip grows. Choosing the nearest
  // center removes dead gaps without making the hit region jump on selection.
  int nearest_index = -1;
  qreal nearest_distance = std::numeric_limits<qreal>::max();
  for (int i = 0; i < pip_rects_.size(); ++i) {
    const qreal distance = qAbs(position.x() - pip_rects_[i].center().x());
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = i;
    }
  }
  return nearest_index >= 0 ? visible_pages_[nearest_index] : -1;
}

}  // namespace autoviz::widget

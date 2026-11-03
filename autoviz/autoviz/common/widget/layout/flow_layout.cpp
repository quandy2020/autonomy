/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/layout/flow_layout.hpp"

#include <QLayoutItem>
#include <QRect>
#include <QSize>
#include <QStyle>
#include <QWidget>
#include <algorithm>

namespace autoviz {
namespace widget {

FlowLayout::FlowLayout(QWidget* parent, int margin, int horizontal_spacing,
                       int vertical_spacing)
    : QLayout(parent),
      horizontal_spacing_(horizontal_spacing),
      vertical_spacing_(vertical_spacing) {
  if (margin >= 0) {
    setContentsMargins(margin, margin, margin, margin);
  }
}

FlowLayout::FlowLayout(int margin, int horizontal_spacing, int vertical_spacing)
    : horizontal_spacing_(horizontal_spacing),
      vertical_spacing_(vertical_spacing) {
  if (margin >= 0) {
    setContentsMargins(margin, margin, margin, margin);
  }
}

FlowLayout::~FlowLayout() {
  while (QLayoutItem* item = takeAt(0)) {
    delete item;
  }
}

void FlowLayout::addItem(QLayoutItem* item) {
  if (item != nullptr) {
    items_.append(item);
  }
}

Qt::Orientations FlowLayout::expandingDirections() const { return {}; }

bool FlowLayout::hasHeightForWidth() const { return true; }

int FlowLayout::heightForWidth(int width) const {
  return DoLayout(QRect(0, 0, width, 0), true);
}

int FlowLayout::count() const { return static_cast<int>(items_.size()); }

QLayoutItem* FlowLayout::itemAt(int index) const {
  return items_.value(index, nullptr);
}

QSize FlowLayout::minimumSize() const {
  QSize size;
  for (const QLayoutItem* item : items_) {
    if (item->isEmpty()) {
      continue;
    }
    size = size.expandedTo(item->minimumSize());
  }

  int left = 0;
  int top = 0;
  int right = 0;
  int bottom = 0;
  getContentsMargins(&left, &top, &right, &bottom);
  size += QSize(left + right, top + bottom);
  return size;
}

void FlowLayout::setGeometry(const QRect& rect) {
  QLayout::setGeometry(rect);
  DoLayout(rect, false);
}

QSize FlowLayout::sizeHint() const { return minimumSize(); }

QLayoutItem* FlowLayout::takeAt(int index) {
  if (index < 0 || index >= items_.size()) {
    return nullptr;
  }
  return items_.takeAt(index);
}

int FlowLayout::horizontalSpacing() const {
  if (horizontal_spacing_ >= 0) {
    return horizontal_spacing_;
  }
  return SmartSpacing(QStyle::PM_LayoutHorizontalSpacing);
}

int FlowLayout::verticalSpacing() const {
  if (vertical_spacing_ >= 0) {
    return vertical_spacing_;
  }
  return SmartSpacing(QStyle::PM_LayoutVerticalSpacing);
}

int FlowLayout::DoLayout(const QRect& rect, bool test_only) const {
  int left = 0;
  int top = 0;
  int right = 0;
  int bottom = 0;
  getContentsMargins(&left, &top, &right, &bottom);
  const QRect effective_rect = rect.adjusted(left, top, -right, -bottom);

  const int horizontal_spacing = std::max(0, horizontalSpacing());
  const int vertical_spacing = std::max(0, verticalSpacing());
  int x = effective_rect.x();
  int y = effective_rect.y();
  int line_height = 0;

  for (QLayoutItem* item : items_) {
    if (item->isEmpty()) {
      continue;
    }
    const QSize item_size = item->sizeHint();
    const int next_x = x + item_size.width() + horizontal_spacing;
    if (next_x - horizontal_spacing > effective_rect.right() &&
        line_height > 0) {
      x = effective_rect.x();
      y += line_height + vertical_spacing;
      line_height = 0;
    }

    if (!test_only) {
      item->setGeometry(QRect(QPoint(x, y), item_size));
    }
    x += item_size.width() + horizontal_spacing;
    line_height = std::max(line_height, item_size.height());
  }

  return y + line_height - rect.y() + bottom;
}

int FlowLayout::SmartSpacing(QStyle::PixelMetric metric) const {
  QObject* owner = parent();
  if (owner == nullptr) {
    return -1;
  }
  if (owner->isWidgetType()) {
    return static_cast<QWidget*>(owner)->style()->pixelMetric(metric);
  }
  return static_cast<QLayout*>(owner)->spacing();
}

}  // namespace widget
}  // namespace autoviz

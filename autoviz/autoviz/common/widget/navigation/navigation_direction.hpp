/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QtCore/qnamespace.h>

namespace autoviz::widget::internal {

inline int HorizontalKeyStep(int key, Qt::LayoutDirection direction) {
  if (key != Qt::Key_Left && key != Qt::Key_Right) {
    return 0;
  }
  const int visual_step = key == Qt::Key_Left ? -1 : 1;
  return direction == Qt::RightToLeft ? -visual_step : visual_step;
}

inline int VisualIndex(int logical_index, int count,
                       Qt::LayoutDirection direction) {
  return direction == Qt::RightToLeft ? count - logical_index - 1
                                      : logical_index;
}

}  // namespace autoviz::widget::internal

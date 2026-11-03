/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Color cycle inspired by topic_tools colorCategory20.
 *****************************************************************************/

#pragma once

#include <cstddef>

#include <QColor>

namespace autoviz {
namespace display {
namespace viz {

/** @brief Returns a categorical color for @p index (cycles every 20). */
inline QColor ColorCategory20(std::size_t index) {
  static const QColor kColors[20] = {
      QColor(31, 119, 180),  QColor(174, 199, 232), QColor(255, 127, 14),
      QColor(255, 187, 120), QColor(44, 160, 44),   QColor(152, 223, 138),
      QColor(214, 39, 40),   QColor(255, 152, 150), QColor(148, 103, 189),
      QColor(197, 176, 213), QColor(140, 86, 75),   QColor(196, 156, 148),
      QColor(227, 119, 194), QColor(247, 182, 210), QColor(127, 127, 127),
      QColor(199, 199, 199), QColor(188, 189, 34),  QColor(219, 219, 141),
      QColor(23, 190, 207),  QColor(158, 218, 229)};
  return kColors[index % 20];
}

}  // namespace viz
}  // namespace display
}  // namespace autoviz

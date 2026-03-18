/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <QWidget>

class QTreeView;

namespace autoview {
namespace common {
namespace properties {


/// Provides a sliding column separator between widgets.
/**
 * A tall skinny invisible widget providing left-right sliding column separator
 * adjustment for a two-column QTreeView via mouse drags.
 * Shows splitter cursor when mouse hovers over it.
 * Uses event filtering to catch resize events for the parent.
 */
class SplitterHandle : public QWidget
{
  Q_OBJECT

public:
  explicit SplitterHandle(QTreeView * parent = 0);

  /// Set the ratio of the parent's left column to the parent widget width.
  void setRatio(float ratio);

  /// Get the ratio of the parent's left column to the parent widget width.
  float getRatio();

  /// Catch resize events sent to parent to update splitter's geometry.
  /**
   * Always returns false.
   */
  bool eventFilter(QObject * event_target, QEvent * event) override;

  void setColor(QColor color) {color_ = color; update();}

  QColor getColor() const {return color_;}

protected:
  void mousePressEvent(QMouseEvent * event) override;

  void mouseMoveEvent(QMouseEvent * event) override;

  void paintEvent(QPaintEvent * event) override;

private:
  /// Update the parent's column widths and this splitter's geometry.
  void updateGeometry();

  QTreeView * parent_;
  int x_press_offset_;

  /// The ratio of the first column width to the entire widget width.
  /**
   * Preserved during parent widget resize.
   */
  float first_column_size_ratio_;
  QColor color_;
};

}  // namespace properties
}  // namespace common
}  // namespace autoview
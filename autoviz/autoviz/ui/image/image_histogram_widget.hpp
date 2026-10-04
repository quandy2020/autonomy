/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file image_histogram_widget.hpp
 * @brief Compact luminance histogram with draggable colormap range handles.
 */

#pragma once

#include <QWidget>

#include "autoviz/ui/image/image_analysis.hpp"

namespace autoviz {
namespace image {

/**
 * @class ImageHistogramWidget
 * @brief Paints a 256-bin histogram; drag to set value min/max for colormap.
 */
class ImageHistogramWidget : public QWidget {
  Q_OBJECT

 public:
  explicit ImageHistogramWidget(QWidget* parent = nullptr);

  void setHistogram(const ImageHistogram& histogram);
  void setRange(double min_value, double max_value);

  double rangeMin() const { return range_min_; }
  double rangeMax() const { return range_max_; }

 signals:
  /** Emitted when the user finishes adjusting the colormap range. */
  void rangeChanged(double min_value, double max_value);

 protected:
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;

 private:
  enum class DragTarget { kNone, kMin, kMax, kBoth };

  int valueToX(double value) const;
  double xToValue(int x) const;
  DragTarget hitTest(const QPoint& pos) const;

  ImageHistogram histogram_;
  double range_min_ = 0.0;
  double range_max_ = 255.0;
  DragTarget drag_ = DragTarget::kNone;
  int drag_anchor_x_ = 0;
  double drag_anchor_min_ = 0.0;
  double drag_anchor_max_ = 255.0;
};

}  // namespace image
}  // namespace autoviz

/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file image_profile_widget.hpp
 * @brief Compact luminance line-profile chart for the Image panel.
 */

#pragma once

#include <QVector>
#include <QWidget>

namespace autoviz {
namespace image {

/**
 * @class ImageProfileWidget
 * @brief Draws a 1-D intensity profile along a Measure/Profile segment.
 */
class ImageProfileWidget : public QWidget {
  Q_OBJECT

 public:
  explicit ImageProfileWidget(QWidget* parent = nullptr);

  void setSamples(const QVector<double>& samples);
  void clear();

 protected:
  void paintEvent(QPaintEvent* event) override;

 private:
  QVector<double> samples_;
};

}  // namespace image
}  // namespace autoviz

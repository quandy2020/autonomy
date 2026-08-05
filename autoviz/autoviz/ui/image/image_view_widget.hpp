/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QImage>
#include <QPointF>
#include <QWidget>

#include "autoviz/ui/image/image_annotation_parser.hpp"

namespace autoviz {
namespace image {

class ImageViewWidget : public QWidget {
  Q_OBJECT

 public:
  explicit ImageViewWidget(QWidget* parent = nullptr);

  void setBackgroundColor(const QColor& color);
  void setFrame(const QImage& image);
  void setAnnotationLayers(const QVector<ImageAnnotationLayer>& layers);
  void setLabelScale(double scale);
  void resetView();
  void setStatusText(const QString& text);

 signals:
  void pixelClicked(int x, int y);
  void pixelHovered(int x, int y);

 protected:
  void paintEvent(QPaintEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void mouseDoubleClickEvent(QMouseEvent* event) override;

 private:
  QRectF imageDrawRect() const;
  QPoint imagePixelAt(const QPointF& widget_pos, bool* ok) const;
  void updateFitScale();

  QImage frame_;
  QVector<ImageAnnotationLayer> annotation_layers_;
  QColor background_color_ = Qt::black;
  double label_scale_ = 1.0;
  double zoom_scale_ = 1.0;
  double fit_scale_ = 1.0;
  QPointF pan_offset_;
  bool panning_ = false;
  QPoint last_pan_pos_;
  QString status_text_;
};

}  // namespace image
}  // namespace autoviz

/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/image/image_view_widget.hpp"

#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QWheelEvent>

namespace autoviz {
namespace image {

ImageViewWidget::ImageViewWidget(QWidget* parent) : QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  setMinimumSize(160, 120);
  setMouseTracking(true);
  setAttribute(Qt::WA_OpaquePaintEvent, true);
  setAttribute(Qt::WA_NoSystemBackground, true);
  setAutoFillBackground(false);
}

void ImageViewWidget::setBackgroundColor(const QColor& color) {
  background_color_ = color;
  update();
}

void ImageViewWidget::setFrame(const QImage& image) {
  // Keep a client-side 32-bit QImage. QPixmap is X11/MIT-SHM backed and can
  // be punched through by a sibling QOpenGLWidget, which looks like color
  // blocks. Packed 24-bit RGB888 has the same tile-drop artifacts.
  if (image.isNull()) {
    frame_ = QImage();
  } else if (image.format() == QImage::Format_ARGB32_Premultiplied) {
    frame_ = image;
  } else {
    frame_ = image.convertToFormat(QImage::Format_ARGB32_Premultiplied);
  }
  updateFitScale();
  if (isVisible()) {
    update();
  }
}

void ImageViewWidget::setAnnotationLayers(
    const QVector<ImageAnnotationLayer>& layers) {
  annotation_layers_ = layers;
  update();
}

void ImageViewWidget::setLabelScale(double scale) {
  label_scale_ = std::max(scale, 0.1);
  update();
}

void ImageViewWidget::setStatusText(const QString& text) {
  status_text_ = text;
  update();
}

void ImageViewWidget::resetView() {
  pan_offset_ = QPointF(0.0, 0.0);
  zoom_scale_ = 1.0;
  updateFitScale();
  update();
}

void ImageViewWidget::updateFitScale() {
  if (frame_.isNull() || width() <= 0 || height() <= 0) {
    fit_scale_ = 1.0;
    return;
  }
  const double sx = static_cast<double>(width()) / frame_.width();
  const double sy = static_cast<double>(height()) / frame_.height();
  fit_scale_ = std::min(sx, sy);
}

QRectF ImageViewWidget::imageDrawRect() const {
  if (frame_.isNull()) {
    return {};
  }
  const double scale = fit_scale_ * zoom_scale_;
  const QSizeF size(frame_.width() * scale, frame_.height() * scale);
  const QPointF top_left((width() - size.width()) * 0.5 + pan_offset_.x(),
                         (height() - size.height()) * 0.5 + pan_offset_.y());
  return QRectF(top_left, size);
}

QPoint ImageViewWidget::imagePixelAt(const QPointF& widget_pos, bool* ok) const {
  if (ok != nullptr) {
    *ok = false;
  }
  if (frame_.isNull()) {
    return {};
  }
  const QRectF draw_rect = imageDrawRect();
  if (!draw_rect.contains(widget_pos)) {
    return {};
  }
  const double u = (widget_pos.x() - draw_rect.left()) / draw_rect.width();
  const double v = (widget_pos.y() - draw_rect.top()) / draw_rect.height();
  const int x = std::clamp(static_cast<int>(u * frame_.width()), 0,
                           std::max(frame_.width() - 1, 0));
  const int y = std::clamp(static_cast<int>(v * frame_.height()), 0,
                           std::max(frame_.height() - 1, 0));
  if (ok != nullptr) {
    *ok = true;
  }
  return QPoint(x, y);
}

void ImageViewWidget::paintEvent(QPaintEvent* event) {
  QPainter painter(this);
  painter.fillRect(event->rect(), background_color_);

  if (frame_.isNull()) {
    painter.setPen(Qt::gray);
    painter.drawText(rect(), Qt::AlignCenter,
                     status_text_.isEmpty() ? tr("No image") : status_text_);
    return;
  }

  const QRect draw_rect = imageDrawRect().toAlignedRect();
  painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
  painter.drawImage(draw_rect, frame_);

  painter.save();
  painter.setClipRect(draw_rect);
  painter.translate(draw_rect.topLeft());
  painter.scale(draw_rect.width() / frame_.width(),
                draw_rect.height() / frame_.height());

  for (const ImageAnnotationLayer& layer : annotation_layers_) {
    for (const ImageAnnotationPolyline& polyline : layer.polylines) {
      if (polyline.points.size() < 2) {
        continue;
      }
      QPen pen(polyline.outline_color, polyline.thickness);
      pen.setCosmetic(true);
      painter.setPen(pen);
      painter.setBrush(Qt::NoBrush);
      if (polyline.closed) {
        painter.drawPolygon(polyline.points.data(),
                            static_cast<int>(polyline.points.size()));
      } else {
        painter.drawPolyline(polyline.points.data(),
                             static_cast<int>(polyline.points.size()));
      }
    }
    for (const ImageAnnotationPoint& point : layer.points) {
      painter.setPen(Qt::NoPen);
      painter.setBrush(point.color);
      const double radius = point.size * 0.5;
      painter.drawEllipse(point.position, radius, radius);
    }
    for (const ImageAnnotationText& text : layer.texts) {
      QFont font = painter.font();
      font.setPointSizeF(text.font_size * label_scale_);
      painter.setFont(font);
      painter.setPen(text.color);
      painter.drawText(text.position, text.text);
    }
  }
  painter.restore();

  if (!status_text_.isEmpty()) {
    painter.setPen(Qt::white);
    painter.drawText(QRect(8, 8, width() - 16, 24), Qt::AlignLeft | Qt::AlignVCenter,
                     status_text_);
  }
}

void ImageViewWidget::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  updateFitScale();
}

void ImageViewWidget::wheelEvent(QWheelEvent* event) {
  const double factor = event->angleDelta().y() > 0 ? 1.1 : 0.9;
  zoom_scale_ = std::clamp(zoom_scale_ * factor, 0.05, 20.0);
  update();
  event->accept();
}

void ImageViewWidget::mousePressEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    bool ok = false;
    const QPoint pixel = imagePixelAt(event->position(), &ok);
    if (ok) {
      emit pixelClicked(pixel.x(), pixel.y());
    }
  } else if (event->button() == Qt::MiddleButton ||
             event->button() == Qt::RightButton) {
    panning_ = true;
    last_pan_pos_ = event->pos();
  }
  QWidget::mousePressEvent(event);
}

void ImageViewWidget::mouseMoveEvent(QMouseEvent* event) {
  bool ok = false;
  const QPoint pixel = imagePixelAt(event->position(), &ok);
  if (ok) {
    emit pixelHovered(pixel.x(), pixel.y());
  }
  if (panning_) {
    pan_offset_ += event->pos() - last_pan_pos_;
    last_pan_pos_ = event->pos();
    update();
  }
  QWidget::mouseMoveEvent(event);
}

void ImageViewWidget::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() == Qt::MiddleButton ||
      event->button() == Qt::RightButton) {
    panning_ = false;
  }
  QWidget::mouseReleaseEvent(event);
}

void ImageViewWidget::mouseDoubleClickEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    resetView();
  }
  QWidget::mouseDoubleClickEvent(event);
}

}  // namespace image
}  // namespace autoviz

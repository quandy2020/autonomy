/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/image/image_view_widget.hpp"

#include <QContextMenuEvent>
#include <QKeyEvent>
#include <QMenu>
#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

#include "autoviz/ui/image/image_analysis.hpp"

namespace autoviz {
namespace image {
namespace {

QString FormatProbeText(const QImage& frame, const QPoint& pixel) {
  if (frame.isNull() || pixel.x() < 0 || pixel.y() < 0 ||
      pixel.x() >= frame.width() || pixel.y() >= frame.height()) {
    return {};
  }
  const QRgb rgba = frame.pixel(pixel);
  const int r = qRed(rgba);
  const int g = qGreen(rgba);
  const int b = qBlue(rgba);
  const int a = qAlpha(rgba);
  const int gray = qGray(rgba);
  if (r == g && g == b) {
    return QStringLiteral("(%1, %2)  I=%3  A=%4")
        .arg(pixel.x())
        .arg(pixel.y())
        .arg(gray)
        .arg(a);
  }
  return QStringLiteral("(%1, %2)  RGB(%3, %4, %5)  A=%6")
      .arg(pixel.x())
      .arg(pixel.y())
      .arg(r)
      .arg(g)
      .arg(b)
      .arg(a);
}

}  // namespace

ImageViewWidget::ImageViewWidget(QWidget* parent) : QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  setMinimumSize(160, 120);
  setMouseTracking(true);
  setContextMenuPolicy(Qt::DefaultContextMenu);
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
  if (probe_valid_) {
    updateProbe(probe_pixel_, true);
  }
  updateFitScale();
  if (isVisible()) {
    update();
  }
}

QImage ImageViewWidget::frame() const { return frame_; }

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
  hud_.title = text;
  hud_.meta.clear();
  update();
}

void ImageViewWidget::setHud(const ImageViewHud& hud) {
  hud_ = hud;
  update();
}

void ImageViewWidget::setTool(ImageViewTool tool) {
  if (tool_ == tool) {
    return;
  }
  tool_ = tool;
  clearAnalysisOverlays();
  setCursor(tool_ == ImageViewTool::kProbe ? Qt::ArrowCursor : Qt::CrossCursor);
  update();
}

void ImageViewWidget::setMeasureCalibration(const CameraIntrinsics& intrinsics,
                                            double plane_depth_m) {
  measure_intrinsics_ = intrinsics;
  measure_depth_m_ = std::max(plane_depth_m, 1e-3);
  refreshAnalysisText();
  update();
}

void ImageViewWidget::clearAnalysisOverlays() {
  measure_has_a_ = false;
  measure_complete_ = false;
  measure_a_ = {};
  measure_b_ = {};
  roi_dragging_ = false;
  roi_rect_ = {};
  analysis_text_.clear();
  update();
}

QRect ImageViewWidget::analysisRoi() const {
  return roi_rect_.isValid() ? roi_rect_ : QRect();
}

void ImageViewWidget::setAnalysisText(const QString& text) {
  analysis_text_ = text;
  update();
}

QPointF ImageViewWidget::widgetPosForPixel(const QPoint& pixel) const {
  const QRectF image_rect = imageDrawRect();
  if (frame_.isNull() || image_rect.isEmpty()) {
    return {};
  }
  return QPointF(
      image_rect.left() +
          (pixel.x() + 0.5) * image_rect.width() / frame_.width(),
      image_rect.top() +
          (pixel.y() + 0.5) * image_rect.height() / frame_.height());
}

void ImageViewWidget::refreshAnalysisText() {
  if ((tool_ == ImageViewTool::kMeasure || tool_ == ImageViewTool::kProfile) &&
      measure_has_a_ && (measure_complete_ || probe_valid_)) {
    const QPoint b = measure_complete_ ? measure_b_ : probe_pixel_;
    const double px = pixelDistance(measure_a_, b);
    if (tool_ == ImageViewTool::kMeasure) {
      const auto meters =
          metricDistance(measure_intrinsics_, measure_a_, b, measure_depth_m_);
      analysis_text_ = formatMeasureReadout(px, meters);
    } else {
      analysis_text_ =
          QStringLiteral("Profile: %1 px").arg(px, 0, 'f', 1);
    }
  } else if (tool_ == ImageViewTool::kRoi &&
             (roi_dragging_ || roi_rect_.isValid())) {
    const QRect rect = roi_dragging_
                           ? QRect(roi_origin_, roi_current_).normalized()
                           : roi_rect_;
    analysis_text_ = QStringLiteral("ROI %1×%2 @ (%3,%4)")
                         .arg(rect.width())
                         .arg(rect.height())
                         .arg(rect.x())
                         .arg(rect.y());
  }
}

QString ImageViewWidget::annotationTooltipAt(const QPoint& pixel) const {
  constexpr double kHitPx = 8.0;
  double best = kHitPx * kHitPx;
  QString tip;
  auto consider = [&](const QPointF& pos, const QString& text) {
    if (text.isEmpty()) {
      return;
    }
    const double dx = pos.x() - pixel.x();
    const double dy = pos.y() - pixel.y();
    const double d2 = dx * dx + dy * dy;
    if (d2 <= best) {
      best = d2;
      tip = text;
    }
  };
  for (const ImageAnnotationLayer& layer : annotation_layers_) {
    for (const ImageAnnotationPoint& point : layer.points) {
      consider(point.position, point.tooltip);
    }
    for (const ImageAnnotationText& text : layer.texts) {
      consider(text.position, text.tooltip.isEmpty() ? text.text : text.tooltip);
    }
    for (const ImageAnnotationPolyline& polyline : layer.polylines) {
      if (polyline.tooltip.isEmpty() || polyline.points.isEmpty()) {
        continue;
      }
      for (const QPointF& p : polyline.points) {
        consider(p, polyline.tooltip);
      }
    }
  }
  return tip;
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

void ImageViewWidget::updateProbe(const QPoint& pixel, bool valid) {
  probe_valid_ = valid;
  if (!valid) {
    probe_text_.clear();
    return;
  }
  probe_pixel_ = pixel;
  probe_text_ = FormatProbeText(frame_, pixel);
}

void ImageViewWidget::paintAnnotations(QPainter* painter) const {
  if (painter == nullptr) {
    return;
  }
  for (const ImageAnnotationLayer& layer : annotation_layers_) {
    for (const ImageAnnotationPolyline& polyline : layer.polylines) {
      if (polyline.points.size() < 2) {
        continue;
      }
      QPen pen(polyline.outline_color, polyline.thickness);
      pen.setCosmetic(true);
      painter->setPen(pen);
      painter->setBrush(Qt::NoBrush);
      if (polyline.closed) {
        painter->drawPolygon(polyline.points.data(),
                             static_cast<int>(polyline.points.size()));
      } else {
        painter->drawPolyline(polyline.points.data(),
                              static_cast<int>(polyline.points.size()));
      }
    }
    for (const ImageAnnotationPoint& point : layer.points) {
      painter->setPen(Qt::NoPen);
      painter->setBrush(point.color);
      const double radius = point.size * 0.5;
      painter->drawEllipse(point.position, radius, radius);
    }
    for (const ImageAnnotationText& text : layer.texts) {
      QFont font = painter->font();
      font.setPointSizeF(text.font_size * label_scale_);
      painter->setFont(font);
      painter->setPen(text.color);
      painter->drawText(text.position, text.text);
    }
  }
}

QImage ImageViewWidget::renderExportImage(bool with_annotations) const {
  if (frame_.isNull()) {
    return {};
  }
  if (!with_annotations || annotation_layers_.isEmpty()) {
    return frame_.convertToFormat(QImage::Format_ARGB32);
  }
  QImage out = frame_.convertToFormat(QImage::Format_ARGB32);
  QPainter painter(&out);
  painter.setRenderHint(QPainter::Antialiasing, true);
  paintAnnotations(&painter);
  painter.end();
  return out;
}

void ImageViewWidget::paintEvent(QPaintEvent* event) {
  QPainter painter(this);
  painter.fillRect(event->rect(), background_color_);

  if (frame_.isNull()) {
    painter.setPen(Qt::gray);
    const QString empty =
        hud_.title.isEmpty() ? tr("No image") : hud_.title;
    painter.drawText(rect(), Qt::AlignCenter, empty);
    return;
  }

  const QRect draw_rect = imageDrawRect().toAlignedRect();
  painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
  painter.drawImage(draw_rect, frame_);

  painter.save();
  painter.setClipRect(draw_rect);
  painter.translate(draw_rect.topLeft());
  painter.scale(draw_rect.width() / static_cast<double>(frame_.width()),
                draw_rect.height() / static_cast<double>(frame_.height()));
  paintAnnotations(&painter);
  painter.restore();

  // Probe crosshair in widget space.
  if (probe_valid_ && !frame_.isNull() && tool_ == ImageViewTool::kProbe) {
    const QPointF center = widgetPosForPixel(probe_pixel_);
    painter.setPen(QPen(QColor(255, 255, 255, 220), 1.0));
    painter.drawLine(QPointF(center.x() - 8.0, center.y()),
                     QPointF(center.x() + 8.0, center.y()));
    painter.drawLine(QPointF(center.x(), center.y() - 8.0),
                     QPointF(center.x(), center.y() + 8.0));
  }

  // Measure / Profile overlay.
  if ((tool_ == ImageViewTool::kMeasure || tool_ == ImageViewTool::kProfile) &&
      measure_has_a_) {
    const QPoint tip = measure_complete_ ? measure_b_
                       : (probe_valid_ ? probe_pixel_ : measure_a_);
    const QPointF wa = widgetPosForPixel(measure_a_);
    const QPointF wb = widgetPosForPixel(tip);
    const QColor stroke = tool_ == ImageViewTool::kProfile
                              ? QColor(167, 139, 250)
                              : QColor(251, 191, 36);
    painter.setPen(QPen(stroke, 2.0));
    painter.drawLine(wa, wb);
    painter.setBrush(stroke);
    painter.drawEllipse(wa, 3.5, 3.5);
    painter.drawEllipse(wb, 3.5, 3.5);
  }

  // ROI overlay.
  if (tool_ == ImageViewTool::kRoi || roi_rect_.isValid()) {
    QRect image_roi;
    if (roi_dragging_) {
      image_roi = QRect(roi_origin_, roi_current_).normalized();
    } else if (roi_rect_.isValid()) {
      image_roi = roi_rect_;
    }
    if (image_roi.isValid() && !image_roi.isEmpty() && !frame_.isNull()) {
      const QRectF image_rect = imageDrawRect();
      const QRectF widget_roi(
          image_rect.left() +
              image_roi.left() * image_rect.width() / frame_.width(),
          image_rect.top() +
              image_roi.top() * image_rect.height() / frame_.height(),
          image_roi.width() * image_rect.width() / frame_.width(),
          image_roi.height() * image_rect.height() / frame_.height());
      painter.fillRect(widget_roi, QColor(56, 189, 248, 40));
      painter.setPen(QPen(QColor(56, 189, 248), 1.5, Qt::DashLine));
      painter.drawRect(widget_roi);
    }
  }

  // Top HUD.
  int hud_y = 8;
  auto draw_hud_line = [&](const QString& line) {
    if (line.isEmpty()) {
      return;
    }
    const QFontMetrics metrics(painter.font());
    const QRect text_rect = metrics.boundingRect(line);
    const QRect bg(6, hud_y - 2, text_rect.width() + 10, text_rect.height() + 4);
    painter.fillRect(bg, QColor(0, 0, 0, 140));
    painter.setPen(Qt::white);
    painter.drawText(QRect(10, hud_y, width() - 20, text_rect.height() + 2),
                     Qt::AlignLeft | Qt::AlignVCenter, line);
    hud_y += text_rect.height() + 6;
  };
  draw_hud_line(hud_.title);
  draw_hud_line(hud_.meta);

  // Bottom readouts: analysis then probe.
  int bottom_y = height() - 12;
  auto draw_bottom_line = [&](const QString& line) {
    if (line.isEmpty()) {
      return;
    }
    const QFontMetrics metrics(painter.font());
    const QRect text_rect = metrics.boundingRect(line);
    bottom_y -= text_rect.height();
    const QRect bg(6, bottom_y - 2, text_rect.width() + 10,
                   text_rect.height() + 4);
    painter.fillRect(bg, QColor(0, 0, 0, 160));
    painter.setPen(Qt::white);
    painter.drawText(QRect(10, bottom_y, width() - 20, text_rect.height() + 2),
                     Qt::AlignLeft | Qt::AlignVCenter, line);
    bottom_y -= 6;
  };
  if (tool_ == ImageViewTool::kProbe && probe_valid_) {
    draw_bottom_line(probe_text_);
  }
  draw_bottom_line(analysis_text_);
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
    if (!ok) {
      QWidget::mousePressEvent(event);
      return;
    }
    if (tool_ == ImageViewTool::kProbe) {
      emit pixelClicked(pixel.x(), pixel.y());
    } else if (tool_ == ImageViewTool::kMeasure ||
               tool_ == ImageViewTool::kProfile) {
      if (!measure_has_a_ || measure_complete_) {
        measure_has_a_ = true;
        measure_complete_ = false;
        measure_a_ = pixel;
        measure_b_ = pixel;
      } else {
        measure_b_ = pixel;
        measure_complete_ = true;
        const double px = pixelDistance(measure_a_, measure_b_);
        if (tool_ == ImageViewTool::kMeasure) {
          const auto meters = metricDistance(measure_intrinsics_, measure_a_,
                                             measure_b_, measure_depth_m_);
          refreshAnalysisText();
          emit measureCompleted(measure_a_, measure_b_, px,
                                meters.value_or(-1.0));
        } else {
          refreshAnalysisText();
          emit profileCompleted(measure_a_, measure_b_);
        }
      }
      refreshAnalysisText();
      update();
    } else if (tool_ == ImageViewTool::kRoi) {
      roi_dragging_ = true;
      roi_origin_ = pixel;
      roi_current_ = pixel;
      refreshAnalysisText();
      update();
    }
  } else if (event->button() == Qt::MiddleButton) {
    panning_ = true;
    last_pan_pos_ = event->pos();
  }
  QWidget::mousePressEvent(event);
}

void ImageViewWidget::mouseMoveEvent(QMouseEvent* event) {
  bool ok = false;
  const QPoint pixel = imagePixelAt(event->position(), &ok);
  updateProbe(pixel, ok);
  if (ok && tool_ == ImageViewTool::kProbe) {
    emit pixelHovered(pixel.x(), pixel.y());
    const QString tip = annotationTooltipAt(pixel);
    if (!tip.isEmpty()) {
      setToolTip(tip);
    } else {
      setToolTip(QString());
    }
  }
  if ((tool_ == ImageViewTool::kMeasure || tool_ == ImageViewTool::kProfile) &&
      measure_has_a_ && !measure_complete_ && ok) {
    measure_b_ = pixel;
    refreshAnalysisText();
  }
  if (tool_ == ImageViewTool::kRoi && roi_dragging_ && ok) {
    roi_current_ = pixel;
    refreshAnalysisText();
  }
  if (panning_) {
    pan_offset_ += event->pos() - last_pan_pos_;
    last_pan_pos_ = event->pos();
  }
  update();
  QWidget::mouseMoveEvent(event);
}

void ImageViewWidget::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() == Qt::MiddleButton) {
    panning_ = false;
  } else if (event->button() == Qt::LeftButton &&
             tool_ == ImageViewTool::kRoi && roi_dragging_) {
    roi_dragging_ = false;
    bool ok = false;
    const QPoint pixel = imagePixelAt(event->position(), &ok);
    if (ok) {
      roi_current_ = pixel;
    }
    roi_rect_ = QRect(roi_origin_, roi_current_).normalized();
    if (roi_rect_.width() < 2 || roi_rect_.height() < 2) {
      roi_rect_ = {};
    }
    refreshAnalysisText();
    emit roiChanged(roi_rect_);
    update();
  }
  QWidget::mouseReleaseEvent(event);
}

void ImageViewWidget::mouseDoubleClickEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    resetView();
  }
  QWidget::mouseDoubleClickEvent(event);
}

void ImageViewWidget::leaveEvent(QEvent* event) {
  updateProbe({}, false);
  update();
  QWidget::leaveEvent(event);
}

void ImageViewWidget::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_1 && !event->modifiers()) {
    resetView();
    event->accept();
    return;
  }
  QWidget::keyPressEvent(event);
}

void ImageViewWidget::contextMenuEvent(QContextMenuEvent* event) {
  QMenu menu(this);
  auto* export_frame =
      menu.addAction(tr("Download image as PNG"));
  auto* export_annotated =
      menu.addAction(tr("Download image with annotations as PNG"));
  menu.addSeparator();
  auto* clear_analysis = menu.addAction(tr("Clear measure / ROI"));
  auto* reset = menu.addAction(tr("Reset view"));
  export_frame->setEnabled(!frame_.isNull());
  export_annotated->setEnabled(!frame_.isNull());

  QAction* chosen = menu.exec(event->globalPos());
  if (chosen == export_frame) {
    emit exportPngRequested(false);
  } else if (chosen == export_annotated) {
    emit exportPngRequested(true);
  } else if (chosen == clear_analysis) {
    clearAnalysisOverlays();
    emit roiChanged({});
  } else if (chosen == reset) {
    resetView();
  }
  event->accept();
}

}  // namespace image
}  // namespace autoviz

/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file image_view_widget.hpp
 * @brief Pan/zoom image canvas with annotation overlay, pixel probe, and HUD.
 *
 * Core view used by @ref ImagePanel and @ref DisplayImageWindow. Fits the
 * frame to the widget, supports wheel zoom, drag pan, pixel probe on hover,
 * PNG export via context menu, and emits pixel coordinates for click/hover
 * publishing.
 *
 * @see ImagePanel
 * @see ImageAnnotationLayer
 */

#pragma once

#include <QColor>
#include <QImage>
#include <QPointF>
#include <QString>
#include <QWidget>

#include "autoviz/ui/image/image_analysis.hpp"
#include "autoviz/ui/image/image_annotation_parser.hpp"
#include "autoviz/ui/image/image_calibration_utils.hpp"

namespace autoviz {
namespace image {

/**
 * @struct ImageViewHud
 * @brief Top-of-view status lines (channel + resolution / FPS / calib).
 */
struct ImageViewHud {
  QString title;  /**< Primary line (e.g. channel name). */
  QString meta;   /**< Secondary line (resolution, FPS, calib flags). */
};

/**
 * @class ImageViewWidget
 * @brief Interactive 2D image viewer with annotation layers and pixel probe.
 *
 * ## Interaction
 *
 * - **Wheel:** zoom toward cursor
 * - **Middle-drag:** pan when zoomed beyond fit
 * - **Double-click / key @c 1:** @ref resetView()
 * - **Right-click:** context menu (export / reset)
 * - **Click / move:** @ref pixelClicked() / @ref pixelHovered()
 * - **Hover:** pixel probe HUD (coordinates + RGBA / luminance)
 *
 * Annotations are drawn in image-pixel space, transformed with the current
 * pan/zoom.
 */
class ImageViewWidget : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs an empty view with a black background.
   *
   * @param parent Qt parent widget.
   */
  explicit ImageViewWidget(QWidget* parent = nullptr);

  /**
   * @brief Sets the letterbox / uncovered background color.
   *
   * @param color Fill color behind the image.
   */
  void setBackgroundColor(const QColor& color);

  /**
   * @brief Replaces the displayed frame and triggers a repaint.
   *
   * @param image RGB (or convertible) frame; null clears the view content.
   */
  void setFrame(const QImage& image);

  /**
   * @brief Returns the current display frame (post-transform composite).
   *
   * @return Copy of the frame used for painting / export.
   */
  QImage frame() const;

  /**
   * @brief Replaces annotation layers drawn on top of the frame.
   *
   * @param layers Timestamped polyline / point / text batches.
   */
  void setAnnotationLayers(const QVector<ImageAnnotationLayer>& layers);

  /**
   * @brief Scales annotation text relative to the default font size.
   *
   * @param scale Multiplier (1.0 = default).
   */
  void setLabelScale(double scale);

  /**
   * @brief Resets zoom and pan so the image fits the widget.
   */
  void resetView();

  /**
   * @brief Sets optional status text drawn over the view (legacy single line).
   *
   * @param text Status string; empty hides the overlay.
   */
  void setStatusText(const QString& text);

  /**
   * @brief Sets structured HUD lines (title + meta).
   *
   * @param hud Channel / resolution / FPS / calib status.
   */
  void setHud(const ImageViewHud& hud);

  /**
   * @brief Renders the current view (frame + annotations) into an image.
   *
   * @param with_annotations When @c false, exports the frame only.
   * @return Image-sized bitmap suitable for PNG export.
   */
  QImage renderExportImage(bool with_annotations) const;

  /**
   * @brief Sets the active interaction tool.
   *
   * @param tool Probe / Measure / ROI.
   */
  void setTool(ImageViewTool tool);

  /** @return Current interaction tool. */
  ImageViewTool tool() const { return tool_; }

  /**
   * @brief Supplies camera intrinsics for metric measure (optional).
   *
   * @param intrinsics Calibration; invalid clears metric mode.
   * @param plane_depth_m Optical-plane depth assumption in meters.
   */
  void setMeasureCalibration(const CameraIntrinsics& intrinsics,
                             double plane_depth_m);

  /** @brief Clears measure endpoints and ROI selection. */
  void clearAnalysisOverlays();

  /**
   * @brief Returns the committed ROI in image pixels (empty if none).
   */
  QRect analysisRoi() const;

  /**
   * @brief Optional analysis status line drawn above the probe HUD.
   *
   * @param text Measure / ROI readout from the panel.
   */
  void setAnalysisText(const QString& text);

 signals:
  /**
   * @brief Emitted on primary-button click over a valid image pixel.
   *
   * @param x Image-pixel X.
   * @param y Image-pixel Y.
   */
  void pixelClicked(int x, int y);

  /**
   * @brief Emitted when the cursor moves over a valid image pixel.
   *
   * @param x Image-pixel X.
   * @param y Image-pixel Y.
   */
  void pixelHovered(int x, int y);

  /**
   * @brief Request exporting the current view as PNG.
   *
   * @param with_annotations Include annotation overlay when @c true.
   */
  void exportPngRequested(bool with_annotations);

  /**
   * @brief Emitted when a measure segment is completed (two clicks).
   *
   * @param a First endpoint (image pixels).
   * @param b Second endpoint (image pixels).
   * @param pixels Euclidean pixel distance.
   * @param meters Metric distance when calibration is available; otherwise -1.
   */
  void measureCompleted(QPoint a, QPoint b, double pixels, double meters);

  /**
   * @brief Emitted when a Profile segment is completed (two clicks).
   *
   * @param a First endpoint (image pixels).
   * @param b Second endpoint (image pixels).
   */
  void profileCompleted(QPoint a, QPoint b);

  /**
   * @brief Emitted when an ROI rectangle is finalized.
   *
   * @param roi Axis-aligned ROI in image pixels (may be empty when cleared).
   */
  void roiChanged(QRect roi);

 protected:
  /**
   * @brief Paints background, scaled frame, annotations, HUD, and probe.
   *
   * @param event Paint event.
   */
  void paintEvent(QPaintEvent* event) override;

  /**
   * @brief Recomputes fit scale when the widget size changes.
   *
   * @param event Resize event.
   */
  void resizeEvent(QResizeEvent* event) override;

  /**
   * @brief Zooms toward the cursor position.
   *
   * @param event Wheel event.
   */
  void wheelEvent(QWheelEvent* event) override;

  /**
   * @brief Begins pan or emits @ref pixelClicked().
   *
   * @param event Mouse press event.
   */
  void mousePressEvent(QMouseEvent* event) override;

  /**
   * @brief Updates pan / probe or emits @ref pixelHovered().
   *
   * @param event Mouse move event.
   */
  void mouseMoveEvent(QMouseEvent* event) override;

  /**
   * @brief Ends an active pan gesture.
   *
   * @param event Mouse release event.
   */
  void mouseReleaseEvent(QMouseEvent* event) override;

  /**
   * @brief Double-click resets the view via @ref resetView().
   *
   * @param event Mouse double-click event.
   */
  void mouseDoubleClickEvent(QMouseEvent* event) override;

  /**
   * @brief Clears the pixel probe when the cursor leaves the widget.
   *
   * @param event Leave event.
   */
  void leaveEvent(QEvent* event) override;

  /**
   * @brief Handles @c 1 to reset the view (Foxglove parity).
   *
   * @param event Key event.
   */
  void keyPressEvent(QKeyEvent* event) override;

  /**
   * @brief Right-click menu: export PNG / reset view.
   *
   * @param event Context menu event.
   */
  void contextMenuEvent(QContextMenuEvent* event) override;

 private:
  /**
   * @brief Axis-aligned rectangle where the image is drawn in widget space.
   *
   * @return Draw rect incorporating fit scale, zoom, and pan.
   */
  QRectF imageDrawRect() const;

  /**
   * @brief Maps a widget position to image-pixel coordinates.
   *
   * @param widget_pos Position in widget coordinates.
   * @param ok Set to @c true when the point lands on the image.
   * @return Integer pixel coordinate when @p ok is true.
   */
  QPoint imagePixelAt(const QPointF& widget_pos, bool* ok) const;

  /** Updates @c fit_scale_ so the frame fits the current widget size. */
  void updateFitScale();

  /**
   * @brief Samples @c frame_ at @p pixel and refreshes the probe HUD string.
   *
   * @param pixel Image-pixel coordinate.
   * @param valid Whether the cursor is over the image.
   */
  void updateProbe(const QPoint& pixel, bool valid);

  /**
   * @brief Draws annotation primitives in image-pixel space onto @p painter.
   *
   * Painter must already be transformed into image-pixel coordinates.
   *
   * @param painter Active painter.
   */
  void paintAnnotations(QPainter* painter) const;

  /** Maps image pixel to widget coordinates (center of pixel). */
  QPointF widgetPosForPixel(const QPoint& pixel) const;

  /** Rebuilds measure / ROI analysis text when endpoints change. */
  void refreshAnalysisText();

  /** Finds nearest annotation tooltip under @p pixel (image space). */
  QString annotationTooltipAt(const QPoint& pixel) const;

  QImage frame_;  /**< Current displayed frame. */
  QVector<ImageAnnotationLayer> annotation_layers_;  /**< Overlay annotations. */
  QColor background_color_ = Qt::black;  /**< Letterbox fill. */
  double label_scale_ = 1.0;   /**< Annotation text scale. */
  double zoom_scale_ = 1.0;    /**< User zoom relative to fit. */
  double fit_scale_ = 1.0;     /**< Scale that fits frame to widget. */
  QPointF pan_offset_;         /**< Pan translation in widget pixels. */
  bool panning_ = false;       /**< Middle-button drag pan in progress. */
  QPoint last_pan_pos_;        /**< Last cursor pos during pan. */
  ImageViewHud hud_;          /**< Top status lines. */
  bool probe_valid_ = false;   /**< Cursor over a valid image pixel. */
  QPoint probe_pixel_;         /**< Image-pixel under cursor. */
  QString probe_text_;         /**< Formatted probe readout. */
  QString analysis_text_;      /**< Measure / ROI status line. */
  ImageViewTool tool_ = ImageViewTool::kProbe;  /**< Active tool. */

  bool measure_has_a_ = false;   /**< Measure first point set. */
  bool measure_complete_ = false; /**< Measure segment finished. */
  QPoint measure_a_;             /**< Measure start. */
  QPoint measure_b_;             /**< Measure end / rubber-band tip. */
  CameraIntrinsics measure_intrinsics_;  /**< Optional calib for meters. */
  double measure_depth_m_ = 1.0; /**< Plane depth for metric measure. */

  bool roi_dragging_ = false;    /**< ROI drag in progress. */
  QPoint roi_origin_;            /**< ROI press corner (image px). */
  QPoint roi_current_;           /**< ROI drag corner (image px). */
  QRect roi_rect_;               /**< Committed ROI (empty = none). */
};

}  // namespace image
}  // namespace autoviz

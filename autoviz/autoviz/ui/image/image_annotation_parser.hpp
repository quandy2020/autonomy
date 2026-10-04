/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file image_annotation_parser.hpp
 * @brief Annotation primitives and payload parser for the Image panel overlay.
 *
 * Converts supported annotation message payloads into drawable polylines,
 * points, and text labels in image-pixel coordinates. Consumed by
 * @ref ImagePanel when merging annotation / projected-marker layers onto the
 * rendered frame.
 *
 * @see ImagePanel
 * @see image_marker_projection.hpp
 */

#pragma once

#include <QColor>
#include <QPointF>
#include <QString>
#include <QVector>

namespace autoviz {
namespace image {

/**
 * @struct ImageAnnotationPolyline
 * @brief Open or closed polyline drawn in image-pixel space.
 */
struct ImageAnnotationPolyline {
  QVector<QPointF> points;  /**< Vertex list in pixel coordinates. */
  QColor outline_color = QColor(QStringLiteral("#00ff88"));  /**< Stroke color. */
  float thickness = 2.0f;   /**< Stroke width in pixels. */
  bool closed = false;      /**< When @c true, connects last vertex back to first. */
  QString tooltip;          /**< Optional hover metadata text. */
};

/**
 * @struct ImageAnnotationPoint
 * @brief Single marker drawn at an image-pixel position.
 */
struct ImageAnnotationPoint {
  QPointF position;  /**< Center in pixel coordinates. */
  QColor color = QColor(QStringLiteral("#ff4444"));  /**< Fill / stroke color. */
  float size = 4.0f;  /**< Marker diameter in pixels. */
  QString tooltip;    /**< Optional hover metadata text. */
};

/**
 * @struct ImageAnnotationText
 * @brief Text label anchored at an image-pixel position.
 */
struct ImageAnnotationText {
  QPointF position;   /**< Baseline / anchor in pixel coordinates. */
  QString text;       /**< UTF-8 label content. */
  QColor color = Qt::white;  /**< Text color. */
  double font_size = 12.0;   /**< Font size in points (scaled by label scale). */
  QString tooltip;    /**< Optional hover metadata text. */
};

/**
 * @struct ImageAnnotationLayer
 * @brief One timestamped batch of annotation primitives for a single channel.
 *
 * Produced by @ref ImageAnnotationParser::fromPayload() or by
 * @ref projectMarkerToLayer(); merged into the view by @ref ImagePanel.
 */
struct ImageAnnotationLayer {
  QVector<ImageAnnotationPolyline> polylines;  /**< Polyline overlays. */
  QVector<ImageAnnotationPoint> points;        /**< Point markers. */
  QVector<ImageAnnotationText> texts;          /**< Text labels. */
  qint64 timestamp_ns = 0;  /**< Source message timestamp (nanoseconds). */
};

/**
 * @class ImageAnnotationParser
 * @brief Stateless parser from typed message payloads to annotation layers.
 *
 * @note All methods are static; no instance state is retained.
 */
class ImageAnnotationParser {
 public:
  /**
   * @brief Parses a serialized annotation message into drawable primitives.
   *
   * @param message_type Fully-qualified protobuf / schema type name.
   * @param payload Serialized message bytes.
   * @return Populated layer (empty on unsupported type or parse failure).
   * @see ImageAnnotationLayer
   */
  static ImageAnnotationLayer fromPayload(const std::string& message_type,
                                          const std::string& payload);
};

}  // namespace image
}  // namespace autoviz

/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QPointF>
#include <QString>
#include <QVector>

namespace autoviz {
namespace image {

struct ImageAnnotationPolyline {
  QVector<QPointF> points;
  QColor outline_color = QColor(QStringLiteral("#00ff88"));
  float thickness = 2.0f;
  bool closed = false;
};

struct ImageAnnotationPoint {
  QPointF position;
  QColor color = QColor(QStringLiteral("#ff4444"));
  float size = 4.0f;
};

struct ImageAnnotationText {
  QPointF position;
  QString text;
  QColor color = Qt::white;
  double font_size = 12.0;
};

struct ImageAnnotationLayer {
  QVector<ImageAnnotationPolyline> polylines;
  QVector<ImageAnnotationPoint> points;
  QVector<ImageAnnotationText> texts;
  qint64 timestamp_ns = 0;
};

class ImageAnnotationParser {
 public:
  static ImageAnnotationLayer fromPayload(const std::string& message_type,
                                          const std::string& payload);
};

}  // namespace image
}  // namespace autoviz

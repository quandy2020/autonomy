/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QPointF>
#include <QString>
#include <QVector>

namespace autoviz {
namespace map {

struct MapGeoPoint {
  double latitude = 0.0;
  double longitude = 0.0;
  double heading_deg = qQNaN();
  double velocity_east_mps = qQNaN();
  double velocity_north_mps = qQNaN();
  quint64 timestamp_ns = 0;
  QString label;
  QColor color;
};

struct MapGeoLine {
  QVector<QPointF> lat_lon_points;
  QColor color = QColor(80, 140, 255);
  float width = 2.0f;
};

struct MapGeoPolygon {
  QVector<QPointF> outer_ring;
  QColor fill_color = QColor(80, 140, 255, 80);
  QColor outline_color = QColor(80, 140, 255);
  float outline_width = 2.0f;
};

struct MapIngestResult {
  QVector<MapGeoPoint> points;
  QVector<MapGeoLine> lines;
  QVector<MapGeoPolygon> polygons;
  bool append_trail = false;
};

class MapGeoJsonParser {
 public:
  static MapIngestResult Parse(const QString& geojson_text, QString* error);
};

}  // namespace map
}  // namespace autoviz

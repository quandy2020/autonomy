/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/map/map_geojson_parser.hpp"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>

namespace autoviz {
namespace map {
namespace {

bool ReadCoordinatePair(const QJsonValue& value, QPointF* out) {
  if (!value.isArray()) {
    return false;
  }
  const QJsonArray array = value.toArray();
  if (array.size() < 2 || !array.at(0).isDouble() || !array.at(1).isDouble()) {
    return false;
  }
  out->setX(array.at(0).toDouble());
  out->setY(array.at(1).toDouble());
  return true;
}

void AppendLineFromCoordinates(const QJsonArray& coordinates, MapIngestResult* result,
                               const QColor& color, float width) {
  MapGeoLine line;
  line.color = color;
  line.width = width;
  for (const QJsonValue& value : coordinates) {
    QPointF point;
    if (ReadCoordinatePair(value, &point)) {
      line.lat_lon_points.push_back(point);
    }
  }
  if (line.lat_lon_points.size() >= 2) {
    result->lines.push_back(line);
  }
}

void AppendPolygonFromCoordinates(const QJsonArray& coordinates,
                                  MapIngestResult* result, const QColor& fill,
                                  const QColor& outline, float outline_width) {
  if (coordinates.isEmpty()) {
    return;
  }
  const QJsonValue outer = coordinates.at(0);
  if (!outer.isArray()) {
    return;
  }
  MapGeoPolygon polygon;
  polygon.fill_color = fill;
  polygon.outline_color = outline;
  polygon.outline_width = outline_width;
  for (const QJsonValue& value : outer.toArray()) {
    QPointF point;
    if (ReadCoordinatePair(value, &point)) {
      polygon.outer_ring.push_back(point);
    }
  }
  if (polygon.outer_ring.size() >= 3) {
    result->polygons.push_back(polygon);
  }
}

void ParseGeometry(const QJsonObject& geometry, MapIngestResult* result) {
  const QString type = geometry.value(QStringLiteral("type")).toString();
  const QJsonValue coordinates = geometry.value(QStringLiteral("coordinates"));
  if (type == QLatin1String("Point")) {
    MapGeoPoint point;
    QPointF lat_lon;
    if (ReadCoordinatePair(coordinates, &lat_lon)) {
      point.longitude = lat_lon.x();
      point.latitude = lat_lon.y();
      result->points.push_back(point);
    }
    return;
  }
  if (type == QLatin1String("MultiPoint") && coordinates.isArray()) {
    for (const QJsonValue& value : coordinates.toArray()) {
      MapGeoPoint point;
      QPointF lat_lon;
      if (ReadCoordinatePair(value, &lat_lon)) {
        point.longitude = lat_lon.x();
        point.latitude = lat_lon.y();
        result->points.push_back(point);
      }
    }
    return;
  }
  if (type == QLatin1String("LineString") && coordinates.isArray()) {
    AppendLineFromCoordinates(coordinates.toArray(), result, QColor(80, 140, 255), 2.0f);
    return;
  }
  if (type == QLatin1String("MultiLineString") && coordinates.isArray()) {
    for (const QJsonValue& line : coordinates.toArray()) {
      if (line.isArray()) {
        AppendLineFromCoordinates(line.toArray(), result, QColor(80, 140, 255), 2.0f);
      }
    }
    return;
  }
  if (type == QLatin1String("Polygon") && coordinates.isArray()) {
    AppendPolygonFromCoordinates(coordinates.toArray(), result,
                                 QColor(80, 140, 255, 80), QColor(80, 140, 255), 2.0f);
    return;
  }
  if (type == QLatin1String("MultiPolygon") && coordinates.isArray()) {
    for (const QJsonValue& polygon : coordinates.toArray()) {
      if (polygon.isArray()) {
        AppendPolygonFromCoordinates(polygon.toArray(), result,
                                     QColor(80, 140, 255, 80), QColor(80, 140, 255),
                                     2.0f);
      }
    }
  }
}

void ParseFeature(const QJsonObject& feature, MapIngestResult* result) {
  const QJsonObject geometry = feature.value(QStringLiteral("geometry")).toObject();
  if (!geometry.isEmpty()) {
    ParseGeometry(geometry, result);
  }
}

}  // namespace

MapIngestResult MapGeoJsonParser::Parse(const QString& geojson_text, QString* error) {
  MapIngestResult result;
  const QJsonDocument doc = QJsonDocument::fromJson(geojson_text.toUtf8());
  if (doc.isNull()) {
    if (error != nullptr) {
      *error = QStringLiteral("Invalid GeoJSON");
    }
    return result;
  }
  if (doc.isObject()) {
    const QJsonObject root = doc.object();
    const QString type = root.value(QStringLiteral("type")).toString();
    if (type == QLatin1String("FeatureCollection")) {
      for (const QJsonValue& value : root.value(QStringLiteral("features")).toArray()) {
        if (value.isObject()) {
          ParseFeature(value.toObject(), &result);
        }
      }
      return result;
    }
    if (type == QLatin1String("Feature")) {
      ParseFeature(root, &result);
      return result;
    }
    if (root.contains(QStringLiteral("coordinates"))) {
      ParseGeometry(root, &result);
      return result;
    }
  }
  if (error != nullptr) {
    *error = QStringLiteral("Unsupported GeoJSON root");
  }
  return result;
}

}  // namespace map
}  // namespace autoviz

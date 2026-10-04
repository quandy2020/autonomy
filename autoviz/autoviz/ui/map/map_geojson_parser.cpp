/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/map/map_geojson_parser.hpp"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>

#include <cmath>

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

QColor CssColor(const QJsonValue& value, double opacity) {
  if (!value.isString()) {
    return {};
  }
  QColor color(value.toString());
  if (!color.isValid()) {
    return {};
  }
  if (opacity >= 0.0 && opacity <= 1.0) {
    color.setAlphaF(opacity);
  }
  return color;
}

double JsonNumber(const QJsonObject& object, const QString& key, double fallback) {
  const QJsonValue value = object.value(key);
  return value.isDouble() ? value.toDouble() : fallback;
}

bool IsPaintProperty(const QString& key) {
  return key == QLatin1String("stroke") || key == QLatin1String("stroke-width") ||
         key == QLatin1String("stroke-opacity") || key == QLatin1String("fill") ||
         key == QLatin1String("fill-opacity") || key == QLatin1String("marker-color") ||
         key == QLatin1String("marker-size") || key == QLatin1String("marker-symbol");
}

QString JsonToText(const QJsonValue& value) {
  if (value.isString()) {
    return value.toString();
  }
  if (value.isBool()) {
    return value.toBool() ? QStringLiteral("true") : QStringLiteral("false");
  }
  if (value.isDouble()) {
    const double number = value.toDouble();
    const double rounded = std::round(number);
    if (std::abs(number - rounded) < 1e-9 && std::abs(number) < 1e15) {
      return QString::number(static_cast<qlonglong>(rounded));
    }
    return QString::number(number, 'g', 8);
  }
  if (value.isArray()) {
    return QString::fromUtf8(
        QJsonDocument(value.toArray()).toJson(QJsonDocument::Compact));
  }
  if (value.isObject()) {
    return QString::fromUtf8(
        QJsonDocument(value.toObject()).toJson(QJsonDocument::Compact));
  }
  return {};
}

QVector<MapAttribute> ReadAttributes(const QJsonObject& properties) {
  QVector<MapAttribute> rows;
  for (auto it = properties.constBegin(); it != properties.constEnd(); ++it) {
    if (IsPaintProperty(it.key()) || it.value().isNull() || it.value().isUndefined()) {
      continue;
    }
    MapAttribute row;
    row.key = it.key();
    row.value = JsonToText(it.value());
    rows.push_back(row);
  }
  return rows;
}

void StampFeatureProperties(const QJsonObject& properties, MapIngestResult* result,
                            int points_before, int lines_before, int polygons_before) {
  if (result == nullptr || properties.isEmpty()) {
    return;
  }
  QString label = properties.value(QStringLiteral("title")).toString();
  if (label.isEmpty()) {
    label = properties.value(QStringLiteral("name")).toString();
  }
  const QString symbol = properties.value(QStringLiteral("marker-symbol")).toString();
  if (symbol.contains(QStringLiteral("rally"), Qt::CaseInsensitive) &&
      !label.contains(QStringLiteral("rally"), Qt::CaseInsensitive)) {
    label = label.isEmpty() ? QStringLiteral("rally") : label + QStringLiteral(" rally");
  }

  const QColor marker = CssColor(properties.value(QStringLiteral("marker-color")), 1.0);
  const double stroke_opacity = JsonNumber(properties, QStringLiteral("stroke-opacity"), 1.0);
  const QColor stroke =
      CssColor(properties.value(QStringLiteral("stroke")), stroke_opacity);
  const double fill_opacity = JsonNumber(properties, QStringLiteral("fill-opacity"), -1.0);
  QColor fill = CssColor(properties.value(QStringLiteral("fill")),
                         fill_opacity >= 0.0 ? fill_opacity : 0.6);
  const double stroke_width = JsonNumber(properties, QStringLiteral("stroke-width"), -1.0);
  const bool has_heading = properties.value(QStringLiteral("heading")).isDouble() ||
                           properties.value(QStringLiteral("bearing")).isDouble();
  const double heading = properties.value(QStringLiteral("heading")).isDouble()
                             ? properties.value(QStringLiteral("heading")).toDouble()
                             : properties.value(QStringLiteral("bearing")).toDouble();
  const QVector<MapAttribute> attributes = ReadAttributes(properties);

  for (int i = points_before; i < result->points.size(); ++i) {
    MapGeoPoint& point = result->points[i];
    point.attributes = attributes;
    if (!label.isEmpty()) {
      point.label = label;
    }
    if (marker.isValid()) {
      point.color = marker;
    }
    if (has_heading) {
      point.heading_deg = heading;
    }
  }
  for (int i = lines_before; i < result->lines.size(); ++i) {
    MapGeoLine& line = result->lines[i];
    line.attributes = attributes;
    line.label = label;
    if (stroke.isValid()) {
      line.color = stroke;
    }
    if (stroke_width > 0.0) {
      line.width = static_cast<float>(stroke_width);
    }
  }
  for (int i = polygons_before; i < result->polygons.size(); ++i) {
    MapGeoPolygon& polygon = result->polygons[i];
    polygon.attributes = attributes;
    polygon.label = label;
    if (stroke.isValid()) {
      polygon.outline_color = stroke;
    }
    if (fill.isValid()) {
      polygon.fill_color = fill;
    }
    if (stroke_width > 0.0) {
      polygon.outline_width = static_cast<float>(stroke_width);
    }
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
    return;
  }
  if (type == QLatin1String("GeometryCollection")) {
    for (const QJsonValue& child : geometry.value(QStringLiteral("geometries")).toArray()) {
      if (child.isObject()) {
        ParseGeometry(child.toObject(), result);
      }
    }
  }
}

void ParseFeature(const QJsonObject& feature, MapIngestResult* result) {
  const int points_before = result->points.size();
  const int lines_before = result->lines.size();
  const int polygons_before = result->polygons.size();
  const QJsonObject geometry = feature.value(QStringLiteral("geometry")).toObject();
  if (!geometry.isEmpty()) {
    ParseGeometry(geometry, result);
  }
  const QJsonValue properties = feature.value(QStringLiteral("properties"));
  if (properties.isObject()) {
    StampFeatureProperties(properties.toObject(), result, points_before, lines_before,
                           polygons_before);
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

/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/map/map_plan.hpp"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <QtMath>

#include <algorithm>
#include <cmath>

namespace autoviz {
namespace map {
namespace {

constexpr double kMetersPerDegreeLat = 110540.0;

struct LocalPoint {
  double x = 0;
  double y = 0;
};

double MetersPerDegreeLon(double latitude) {
  return 111320.0 * std::cos(qDegreesToRadians(latitude));
}

LocalPoint ToLocal(const MapPlanPoint& point, double origin_lat, double origin_lon,
                   double meters_per_lon) {
  LocalPoint local;
  local.x = (point.longitude - origin_lon) * meters_per_lon;
  local.y = (point.latitude - origin_lat) * kMetersPerDegreeLat;
  return local;
}

MapPlanPoint FromLocal(const LocalPoint& local, double origin_lat, double origin_lon,
                       double meters_per_lon) {
  MapPlanPoint point;
  point.latitude = origin_lat + local.y / kMetersPerDegreeLat;
  point.longitude = origin_lon + local.x / std::max(meters_per_lon, 1.0);
  return point;
}

bool CrossesScanLine(const LocalPoint& a, const LocalPoint& b, double y, double* x) {
  if (x == nullptr || a.y == b.y) {
    return false;
  }
  const double y0 = std::min(a.y, b.y);
  const double y1 = std::max(a.y, b.y);
  if (y < y0 || y >= y1) {
    return false;
  }
  const double t = (y - a.y) / (b.y - a.y);
  *x = a.x + t * (b.x - a.x);
  return true;
}

QJsonArray LineCoordinates(const QVector<MapPlanPoint>& points, bool close) {
  QJsonArray coordinates;
  for (const MapPlanPoint& point : points) {
    coordinates.append(QJsonArray{point.longitude, point.latitude});
  }
  if (close && !points.isEmpty()) {
    coordinates.append(QJsonArray{points.front().longitude, points.front().latitude});
  }
  return coordinates;
}

QVector<MapPlanPoint> PointsFromCoordinates(const QJsonArray& coordinates) {
  QVector<MapPlanPoint> points;
  for (const QJsonValue& value : coordinates) {
    const QJsonArray pair = value.toArray();
    if (pair.size() < 2) {
      continue;
    }
    MapPlanPoint point;
    point.longitude = pair.at(0).toDouble();
    point.latitude = pair.at(1).toDouble();
    points.push_back(point);
  }
  if (points.size() >= 2 &&
      std::abs(points.front().latitude - points.back().latitude) < 1e-9 &&
      std::abs(points.front().longitude - points.back().longitude) < 1e-9) {
    points.removeLast();
  }
  return points;
}

}  // namespace

QVector<MapPlanPoint> SurveyLawnmower(const QVector<MapPlanPoint>& fence,
                                      double spacing_m) {
  if (fence.size() < 3 || !(spacing_m > 1.0)) {
    return {};
  }
  const double origin_lat = fence.front().latitude;
  const double origin_lon = fence.front().longitude;
  const double meters_per_lon = std::max(MetersPerDegreeLon(origin_lat), 1.0);
  QVector<LocalPoint> local;
  local.reserve(fence.size());
  double min_y = 0;
  double max_y = 0;
  for (int i = 0; i < fence.size(); ++i) {
    local.push_back(ToLocal(fence.at(i), origin_lat, origin_lon, meters_per_lon));
    min_y = i == 0 ? local.back().y : std::min(min_y, local.back().y);
    max_y = i == 0 ? local.back().y : std::max(max_y, local.back().y);
  }

  QVector<MapPlanPoint> path;
  bool left_to_right = true;
  for (double y = min_y + spacing_m * 0.5; y < max_y; y += spacing_m) {
    QVector<double> hits;
    for (int i = 0; i < local.size(); ++i) {
      double x = 0;
      if (CrossesScanLine(local.at(i), local.at((i + 1) % local.size()), y, &x)) {
        hits.push_back(x);
      }
    }
    std::sort(hits.begin(), hits.end());
    if (hits.size() < 2) {
      continue;
    }
    const double x0 = left_to_right ? hits.front() : hits.back();
    const double x1 = left_to_right ? hits.back() : hits.front();
    path.push_back(FromLocal(LocalPoint{x0, y}, origin_lat, origin_lon, meters_per_lon));
    path.push_back(FromLocal(LocalPoint{x1, y}, origin_lat, origin_lon, meters_per_lon));
    left_to_right = !left_to_right;
  }
  return path;
}

QVector<MapPlanPoint> CorridorScan(const QVector<MapPlanPoint>& centerline,
                                   double width_m) {
  if (centerline.size() < 2 || !(width_m > 1.0)) {
    return {};
  }
  const double origin_lat = centerline.front().latitude;
  const double origin_lon = centerline.front().longitude;
  const double meters_per_lon = std::max(MetersPerDegreeLon(origin_lat), 1.0);
  const double offset = width_m * 0.5;
  QVector<MapPlanPoint> left;
  QVector<MapPlanPoint> right;
  left.reserve(centerline.size());
  right.reserve(centerline.size());
  for (int i = 0; i < centerline.size(); ++i) {
    const int prev = std::max(i - 1, 0);
    const int next = std::min(i + 1, static_cast<int>(centerline.size()) - 1);
    const LocalPoint a =
        ToLocal(centerline.at(prev), origin_lat, origin_lon, meters_per_lon);
    const LocalPoint b =
        ToLocal(centerline.at(next), origin_lat, origin_lon, meters_per_lon);
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length = std::hypot(dx, dy);
    if (length < 1e-3) {
      continue;
    }
    const LocalPoint here =
        ToLocal(centerline.at(i), origin_lat, origin_lon, meters_per_lon);
    const double nx = -dy / length;
    const double ny = dx / length;
    left.push_back(FromLocal(LocalPoint{here.x + nx * offset, here.y + ny * offset},
                             origin_lat, origin_lon, meters_per_lon));
    right.push_back(FromLocal(LocalPoint{here.x - nx * offset, here.y - ny * offset},
                              origin_lat, origin_lon, meters_per_lon));
  }
  QVector<MapPlanPoint> path = centerline;
  for (int i = right.size() - 1; i >= 0; --i) {
    path.push_back(right.at(i));
  }
  for (const MapPlanPoint& point : left) {
    path.push_back(point);
  }
  return path;
}

QVector<MapPlanPoint> StructureScan(double latitude, double longitude,
                                    double radius_m, int count) {
  count = std::max(count, 3);
  if (!(radius_m > 1.0)) {
    return {};
  }
  const double meters_per_lon = std::max(MetersPerDegreeLon(latitude), 1.0);
  QVector<MapPlanPoint> ring;
  ring.reserve(count);
  for (int i = 0; i < count; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(count);
    LocalPoint local;
    local.x = radius_m * std::cos(angle);
    local.y = radius_m * std::sin(angle);
    ring.push_back(FromLocal(local, latitude, longitude, meters_per_lon));
  }
  ring.push_back(ring.front());
  return ring;
}

QByteArray ExportPlanGeoJson(const QVector<MapPlanPoint>& waypoints,
                             const QVector<MapPlanPoint>& geofence,
                             const QVector<MapPlanPoint>& rally_points) {
  QJsonArray features;
  auto add = [&features](const QString& kind, const QString& geometry_type,
                         const QJsonValue& coordinates) {
    if (coordinates.toArray().isEmpty()) {
      return;
    }
    QJsonObject geometry;
    geometry.insert(QStringLiteral("type"), geometry_type);
    geometry.insert(QStringLiteral("coordinates"), coordinates);
    QJsonObject properties;
    properties.insert(QStringLiteral("kind"), kind);
    QJsonObject feature;
    feature.insert(QStringLiteral("type"), QStringLiteral("Feature"));
    feature.insert(QStringLiteral("properties"), properties);
    feature.insert(QStringLiteral("geometry"), geometry);
    features.append(feature);
  };
  add(QStringLiteral("waypoints"), QStringLiteral("LineString"),
      LineCoordinates(waypoints, false));
  QJsonArray rings;
  rings.append(LineCoordinates(geofence, true));
  add(QStringLiteral("geofence"), QStringLiteral("Polygon"), rings);
  add(QStringLiteral("rally"), QStringLiteral("MultiPoint"),
      LineCoordinates(rally_points, false));

  QJsonObject root;
  root.insert(QStringLiteral("type"), QStringLiteral("FeatureCollection"));
  root.insert(QStringLiteral("features"), features);
  return QJsonDocument(root).toJson(QJsonDocument::Indented);
}

bool ImportPlanGeoJson(const QByteArray& bytes, QVector<MapPlanPoint>* waypoints,
                       QVector<MapPlanPoint>* geofence,
                       QVector<MapPlanPoint>* rally_points, QString* error) {
  if (waypoints == nullptr || geofence == nullptr || rally_points == nullptr) {
    return false;
  }
  QJsonParseError parse_error;
  const QJsonDocument document = QJsonDocument::fromJson(bytes, &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !document.isObject()) {
    if (error != nullptr) {
      *error = parse_error.errorString();
    }
    return false;
  }
  const QJsonArray features = document.object().value(QStringLiteral("features")).toArray();
  if (features.isEmpty()) {
    if (error != nullptr) {
      *error = QStringLiteral("GeoJSON has no features");
    }
    return false;
  }
  QVector<MapPlanPoint> next_waypoints;
  QVector<MapPlanPoint> next_fence;
  QVector<MapPlanPoint> next_rally;
  bool found = false;
  for (const QJsonValue& value : features) {
    const QJsonObject feature = value.toObject();
    const QString kind =
        feature.value(QStringLiteral("properties")).toObject().value(QStringLiteral("kind")).toString();
    const QJsonObject geometry = feature.value(QStringLiteral("geometry")).toObject();
    const QString type = geometry.value(QStringLiteral("type")).toString();
    const QJsonArray coordinates = geometry.value(QStringLiteral("coordinates")).toArray();
    if (kind == QStringLiteral("waypoints") && type == QStringLiteral("LineString")) {
      next_waypoints = PointsFromCoordinates(coordinates);
      found = true;
    } else if (kind == QStringLiteral("geofence") && type == QStringLiteral("Polygon") &&
               !coordinates.isEmpty()) {
      next_fence = PointsFromCoordinates(coordinates.at(0).toArray());
      found = true;
    } else if (kind == QStringLiteral("rally") && type == QStringLiteral("MultiPoint")) {
      next_rally = PointsFromCoordinates(coordinates);
      found = true;
    }
  }
  if (!found) {
    if (error != nullptr) {
      *error = QStringLiteral("No waypoint, geofence, or rally features");
    }
    return false;
  }
  *waypoints = next_waypoints;
  *geofence = next_fence;
  *rally_points = next_rally;
  return true;
}

}  // namespace map
}  // namespace autoviz

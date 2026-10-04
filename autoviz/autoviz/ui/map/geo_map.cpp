/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/map/geo_map.hpp"

#include <QFile>
#include <QFileInfo>
#include <QStringList>

#include <QtMath>

#include <algorithm>
#include <cmath>

namespace autoviz {
namespace map {
namespace {

constexpr double kPickPixels = 10.0;

double HaversineMeters(double lat1, double lon1, double lat2, double lon2);

QString LayerDisplayName(const MapRenderLayerSnapshot& layer) {
  return layer.style.channel.isEmpty() ? layer.channel : layer.style.channel;
}

QString FormatGroundLength(double meters) {
  if (meters >= 1000.0) {
    return QString::number(meters / 1000.0, 'f', 2) + QStringLiteral(" km");
  }
  return QString::number(meters, 'f', 0) + QStringLiteral(" m");
}

QString FormatGroundArea(double square_meters) {
  if (square_meters >= 1.0e6) {
    return QString::number(square_meters / 1.0e6, 'f', 2) + QStringLiteral(" km²");
  }
  return QString::number(square_meters, 'f', 0) + QStringLiteral(" m²");
}

double PathLengthMeters(const QVector<QPointF>& lon_lat) {
  double meters = 0.0;
  for (int i = 1; i < lon_lat.size(); ++i) {
    meters += HaversineMeters(lon_lat.at(i - 1).y(), lon_lat.at(i - 1).x(),
                              lon_lat.at(i).y(), lon_lat.at(i).x());
  }
  return meters;
}

double RingAreaSqMeters(const QVector<QPointF>& lon_lat) {
  if (lon_lat.size() < 3) {
    return 0.0;
  }
  const double lat0 = qDegreesToRadians(lon_lat.first().y());
  const double meters_per_lon = 111320.0 * std::cos(lat0);
  constexpr double kMetersPerLat = 110540.0;
  double sum = 0.0;
  for (int i = 0; i < lon_lat.size(); ++i) {
    const QPointF& a = lon_lat.at(i);
    const QPointF& b = lon_lat.at((i + 1) % lon_lat.size());
    sum += a.x() * meters_per_lon * b.y() * kMetersPerLat -
           b.x() * meters_per_lon * a.y() * kMetersPerLat;
  }
  return std::abs(sum) * 0.5;
}

void AppendAttribute(QVector<MapAttribute>* rows, const QString& key, const QString& value) {
  if (rows == nullptr || key.isEmpty() || value.isEmpty()) {
    return;
  }
  for (const MapAttribute& row : *rows) {
    if (row.key == key) {
      return;
    }
  }
  MapAttribute row;
  row.key = key;
  row.value = value;
  rows->push_back(row);
}

double DistanceToSegment(const QPointF& point, const QPointF& a, const QPointF& b) {
  const QPointF ab = b - a;
  const double length2 = ab.x() * ab.x() + ab.y() * ab.y();
  if (length2 < 1e-6) {
    return std::hypot(point.x() - a.x(), point.y() - a.y());
  }
  const double t = std::clamp(((point.x() - a.x()) * ab.x() + (point.y() - a.y()) * ab.y()) /
                                  length2,
                              0.0, 1.0);
  const QPointF closest = a + ab * t;
  return std::hypot(point.x() - closest.x(), point.y() - closest.y());
}

bool PointInPolygon(const QPointF& point, const QVector<QPointF>& ring) {
  bool inside = false;
  for (qsizetype i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
    const QPointF& a = ring.at(i);
    const QPointF& b = ring.at(j);
    const bool crosses = ((a.y() > point.y()) != (b.y() > point.y())) &&
                         (point.x() < (b.x() - a.x()) * (point.y() - a.y()) /
                                              ((b.y() - a.y()) + 1e-12) +
                                          a.x());
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
}

double HaversineMeters(double lat1, double lon1, double lat2, double lon2) {
  constexpr double kEarthRadius = 6378137.0;
  const double d_lat = qDegreesToRadians(lat2 - lat1);
  const double d_lon = qDegreesToRadians(lon2 - lon1);
  const double a = std::sin(d_lat * 0.5) * std::sin(d_lat * 0.5) +
                   std::cos(qDegreesToRadians(lat1)) * std::cos(qDegreesToRadians(lat2)) *
                       std::sin(d_lon * 0.5) * std::sin(d_lon * 0.5);
  return 2.0 * kEarthRadius * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

GeoMapHit HitTestLayers(
    const QPointF& screen, const QVector<MapRenderLayerSnapshot>& layers,
    const std::function<QPointF(double, double)>& to_screen) {
  GeoMapHit best;
  double best_distance = kPickPixels;
  for (const MapRenderLayerSnapshot& layer : layers) {
    if (!layer.style.enabled) {
      continue;
    }
    for (int i = 0; i < layer.points.size(); ++i) {
      const MapGeoPoint& point = layer.points.at(i);
      const QPointF pixel = to_screen(point.latitude, point.longitude);
      const double distance = std::hypot(pixel.x() - screen.x(), pixel.y() - screen.y());
      const double limit = std::max(kPickPixels, layer.style.point_size);
      if (distance <= limit && distance < best_distance) {
        best_distance = distance;
        best.valid = true;
        best.layer_id = layer.channel;
        best.kind = GeoMapFeatureKind::kPoint;
        best.index = i;
        best.latitude = point.latitude;
        best.longitude = point.longitude;
        best.label = point.label.isEmpty() ? LayerDisplayName(layer) : point.label;
        best.layer_name = LayerDisplayName(layer);
        best.attributes = point.attributes;
        if (std::isfinite(point.heading_deg)) {
          AppendAttribute(&best.attributes, QStringLiteral("heading"),
                          QString::number(point.heading_deg, 'f', 1));
        }
        if (std::isfinite(point.velocity_east_mps) ||
            std::isfinite(point.velocity_north_mps)) {
          const double east =
              std::isfinite(point.velocity_east_mps) ? point.velocity_east_mps : 0.0;
          const double north =
              std::isfinite(point.velocity_north_mps) ? point.velocity_north_mps : 0.0;
          AppendAttribute(&best.attributes, QStringLiteral("speed_mps"),
                          QString::number(std::hypot(east, north), 'f', 2));
        }
      }
    }
    for (int i = 0; i < layer.lines.size(); ++i) {
      const MapGeoLine& line = layer.lines.at(i);
      double line_distance = 1e9;
      for (int v = 1; v < line.lat_lon_points.size(); ++v) {
        const QPointF a = to_screen(line.lat_lon_points.at(v - 1).y(),
                                    line.lat_lon_points.at(v - 1).x());
        const QPointF b =
            to_screen(line.lat_lon_points.at(v).y(), line.lat_lon_points.at(v).x());
        line_distance = std::min(line_distance, DistanceToSegment(screen, a, b));
      }
      if (line_distance <= kPickPixels && line_distance < best_distance) {
        best_distance = line_distance;
        best.valid = true;
        best.layer_id = layer.channel;
        best.kind = GeoMapFeatureKind::kLine;
        best.index = i;
        if (!line.lat_lon_points.isEmpty()) {
          best.longitude = line.lat_lon_points.at(line.lat_lon_points.size() / 2).x();
          best.latitude = line.lat_lon_points.at(line.lat_lon_points.size() / 2).y();
        }
        best.label = line.label.isEmpty() ? LayerDisplayName(layer) : line.label;
        best.layer_name = LayerDisplayName(layer);
        best.attributes = line.attributes;
        AppendAttribute(&best.attributes, QStringLiteral("vertices"),
                        QString::number(line.lat_lon_points.size()));
        AppendAttribute(&best.attributes, QStringLiteral("length"),
                        FormatGroundLength(PathLengthMeters(line.lat_lon_points)));
      }
    }
  }
  if (best.valid) {
    return best;
  }
  for (const MapRenderLayerSnapshot& layer : layers) {
    if (!layer.style.enabled) {
      continue;
    }
    for (int i = 0; i < layer.polygons.size(); ++i) {
      const MapGeoPolygon& polygon = layer.polygons.at(i);
      QVector<QPointF> ring;
      ring.reserve(polygon.outer_ring.size());
      for (const QPointF& lat_lon : polygon.outer_ring) {
        ring.push_back(to_screen(lat_lon.y(), lat_lon.x()));
      }
      if (ring.size() >= 3 && PointInPolygon(screen, ring)) {
        best.valid = true;
        best.layer_id = layer.channel;
        best.kind = GeoMapFeatureKind::kPolygon;
        best.index = i;
        best.latitude = polygon.outer_ring.first().y();
        best.longitude = polygon.outer_ring.first().x();
        best.label = polygon.label.isEmpty() ? LayerDisplayName(layer) : polygon.label;
        best.layer_name = LayerDisplayName(layer);
        best.attributes = polygon.attributes;
        AppendAttribute(&best.attributes, QStringLiteral("vertices"),
                        QString::number(polygon.outer_ring.size()));
        AppendAttribute(&best.attributes, QStringLiteral("area"),
                        FormatGroundArea(RingAreaSqMeters(polygon.outer_ring)));
        return best;
      }
    }
  }
  return best;
}

}  // namespace

QString GeoMap::loadFile(const QString& path, QString* error) {
  const QString absolute = QFileInfo(path).absoluteFilePath();
  for (const GeoMapLayer& layer : layers_) {
    if (layer.path == absolute) {
      return layer.id;
    }
  }
  QFile file(absolute);
  if (!file.open(QIODevice::ReadOnly)) {
    GeoMapLayer layer;
    layer.id = QStringLiteral("geo:%1").arg(next_id_++);
    layer.path = absolute;
    layer.source = QFileInfo(absolute).fileName();
    layer.status = GeoMapLayerStatus::kError;
    layer.status_text = QStringLiteral("Cannot read %1").arg(path);
    if (error != nullptr) {
      *error = layer.status_text;
    }
    layers_.push_back(layer);
    return layer.id;
  }
  const QString text = QString::fromUtf8(file.readAll());
  const QString id = loadText(text, QFileInfo(absolute).fileName(), error);
  for (GeoMapLayer& layer : layers_) {
    if (layer.id == id) {
      layer.path = absolute;
      break;
    }
  }
  return id;
}

QString GeoMap::loadText(const QString& text, const QString& source, QString* error) {
  GeoMapLayer layer;
  layer.id = QStringLiteral("geo:%1").arg(next_id_++);
  layer.source = source.isEmpty() ? layer.id : source;
  QString parse_error;
  layer.features = MapGeoJsonParser::Parse(text, &parse_error);
  const bool empty = layer.features.points.isEmpty() && layer.features.lines.isEmpty() &&
                     layer.features.polygons.isEmpty();
  if (!parse_error.isEmpty() && empty) {
    layer.status = GeoMapLayerStatus::kError;
    layer.status_text = parse_error;
    if (error != nullptr) {
      *error = parse_error;
    }
  }
  layers_.push_back(layer);
  return layer.id;
}

bool GeoMap::setLayerVisible(const QString& layer_id, bool visible) {
  for (GeoMapLayer& layer : layers_) {
    if (layer.id == layer_id) {
      layer.visible = visible;
      if (!visible && selection_.layer_id == layer_id) {
        clearSelection();
      }
      return true;
    }
  }
  return false;
}

bool GeoMap::removeLayer(const QString& layer_id) {
  for (int i = 0; i < layers_.size(); ++i) {
    if (layers_.at(i).id != layer_id) {
      continue;
    }
    layers_.removeAt(i);
    if (selection_.layer_id == layer_id) {
      clearSelection();
    }
    return true;
  }
  return false;
}

void GeoMap::syncSources(const QVector<MapGeoJsonSource>& sources) {
  bool unchanged = sources.size() == layers_.size();
  if (unchanged) {
    for (int i = 0; i < sources.size(); ++i) {
      const QString absolute = QFileInfo(sources.at(i).path).absoluteFilePath();
      if (layers_.at(i).path != absolute || layers_.at(i).visible != sources.at(i).visible) {
        unchanged = false;
        break;
      }
    }
  }
  if (unchanged) {
    return;
  }

  QVector<GeoMapLayer> next;
  next.reserve(sources.size());
  for (const MapGeoJsonSource& source : sources) {
    if (source.path.isEmpty()) {
      continue;
    }
    const QString absolute = QFileInfo(source.path).absoluteFilePath();
    const GeoMapLayer* existing = nullptr;
    for (const GeoMapLayer& layer : layers_) {
      if (layer.path == absolute) {
        existing = &layer;
        break;
      }
    }
    if (existing != nullptr) {
      GeoMapLayer layer = *existing;
      layer.visible = source.visible;
      next.push_back(layer);
      continue;
    }
    QString error;
    const QString id = loadFile(absolute, &error);
    for (int i = 0; i < layers_.size(); ++i) {
      if (layers_.at(i).id == id) {
        GeoMapLayer layer = layers_.at(i);
        layer.visible = source.visible;
        next.push_back(layer);
        layers_.removeAt(i);
        break;
      }
    }
  }
  layers_ = next;
  bool selection_kept = false;
  for (const GeoMapLayer& layer : layers_) {
    if (layer.id == selection_.layer_id) {
      selection_kept = true;
      break;
    }
  }
  if (!selection_kept) {
    clearSelection();
  }
}

QVector<MapGeoJsonSource> GeoMap::sources() const {
  QVector<MapGeoJsonSource> sources;
  for (const GeoMapLayer& layer : layers_) {
    if (layer.path.isEmpty()) {
      continue;
    }
    MapGeoJsonSource source;
    source.path = layer.path;
    source.visible = layer.visible;
    sources.push_back(source);
  }
  return sources;
}

QVector<MapRenderLayerSnapshot> GeoMap::renderLayers() const {
  QVector<MapRenderLayerSnapshot> snapshots;
  for (const GeoMapLayer& layer : layers_) {
    if (!layer.visible || layer.status != GeoMapLayerStatus::kOk) {
      continue;
    }
    MapRenderLayerSnapshot snapshot;
    snapshot.channel = layer.id;
    snapshot.style.channel = layer.source;
    snapshot.style.enabled = true;
    snapshot.style.color = QColor(37, 99, 235);
    snapshot.points = layer.features.points;
    snapshot.lines = layer.features.lines;
    snapshot.polygons = layer.features.polygons;
    snapshots.push_back(snapshot);
  }
  return snapshots;
}

GeoMapHit GeoMap::hitTest(
    const QPointF& screen, const QVector<MapRenderLayerSnapshot>& live_layers,
    const std::function<QPointF(double, double)>& to_screen) const {
  if (!to_screen) {
    return {};
  }
  QVector<MapRenderLayerSnapshot> layers = live_layers;
  layers += renderLayers();
  return HitTestLayers(screen, layers, to_screen);
}

void GeoMap::setMeasuring(bool enabled) {
  measuring_ = enabled;
  if (!enabled) {
    measure_points_.clear();
  }
}

void GeoMap::addMeasurePoint(double latitude, double longitude) {
  if (measure_points_.size() >= 2) {
    measure_points_.clear();
  }
  measure_points_.push_back(QPointF(latitude, longitude));
}

void GeoMap::clearMeasure() {
  measure_points_.clear();
  measuring_ = false;
}

double GeoMap::measureMeters() const {
  if (measure_points_.size() < 2) {
    return 0.0;
  }
  const QPointF& a = measure_points_.at(0);
  const QPointF& b = measure_points_.at(1);
  return HaversineMeters(a.x(), a.y(), b.x(), b.y());
}

QString GeoMap::formatMeasure(MapDistanceUnit unit) const {
  if (measure_points_.size() < 2) {
    return {};
  }
  const double meters = measureMeters();
  if (unit == MapDistanceUnit::kFeet) {
    const double feet = meters * 3.2808399;
    if (feet >= 5280.0) {
      const double miles = feet / 5280.0;
      return QString::number(miles, 'f', miles >= 10.0 ? 0 : 1) +
             QStringLiteral(" mi");
    }
    return QString::number(feet, 'f', 0) + QStringLiteral(" ft");
  }
  if (meters >= 1000.0) {
    return QString::number(meters / 1000.0, 'f', 2) + QStringLiteral(" km");
  }
  return QString::number(meters, 'f', 0) + QStringLiteral(" m");
}

QString GeoMap::statusLine(MapDistanceUnit unit) const {
  QStringList parts;
  if (selection_.valid) {
    parts << selection_.label;
  }
  const QString measure = formatMeasure(unit);
  if (!measure.isEmpty()) {
    parts << measure;
  }
  for (const GeoMapLayer& layer : layers_) {
    if (layer.status == GeoMapLayerStatus::kError) {
      parts << layer.source + QStringLiteral(": ") + layer.status_text;
    }
  }
  return parts.join(QStringLiteral("  ·  "));
}

}  // namespace map
}  // namespace autoviz

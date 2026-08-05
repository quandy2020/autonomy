/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/map/map_message_ingest.hpp"

#include <QHash>
#include <QPointF>

#include <QtMath>

#include <automsgs/msgs/sensor_msgs/nav_sat_fix.pb.h>
#include <automsgs/msgs/std_msgs/string.pb.h>
#include <automsgs/msgs/strata_msgs/poi_marker.pb.h>
#include <automsgs/msgs/strata_msgs/robot_marker.pb.h>
#include <automsgs/msgs/strata_msgs/road_graph.pb.h>
#include <automsgs/msgs/strata_msgs/semantic_zone.pb.h>

#include "autoviz/commsgs/message_type_utils.hpp"
#include "autoviz/commsgs/time_utils.hpp"

namespace autoviz {
namespace map {
namespace {

QString NormalizeType(const QString& message_type) {
  return QString::fromStdString(
      commsgs::NormalizeMessageType(message_type.toStdString()));
}

QColor ColorFromRgba(float r, float g, float b, float a = 1.0f) {
  return QColor(static_cast<int>(r * 255.0f), static_cast<int>(g * 255.0f),
                static_cast<int>(b * 255.0f), static_cast<int>(a * 255.0f));
}

MapGeoPoint PointFromLngLat(const automsgs::msgs::geometry_msgs::Point& point,
                            quint64 timestamp_ns, const QString& label = {}) {
  MapGeoPoint geo_point;
  geo_point.longitude = point.x();
  geo_point.latitude = point.y();
  geo_point.timestamp_ns = timestamp_ns;
  geo_point.label = label;
  return geo_point;
}

MapIngestResult IngestNavSatFix(const std::string& payload) {
  MapIngestResult result;
  result.append_trail = true;
  automsgs::msgs::sensor_msgs::NavSatFix message;
  if (!message.ParseFromString(payload)) {
    return result;
  }
  MapGeoPoint point;
  point.latitude = message.latitude();
  point.longitude = message.longitude();
  point.timestamp_ns = commsgs::TimeToNanoseconds(message.header().stamp());
  result.points.push_back(point);
  return result;
}

MapIngestResult IngestRobotMarkerArray(const std::string& payload) {
  MapIngestResult result;
  automsgs::msgs::strata_msgs::RobotMarkerArray message;
  if (!message.ParseFromString(payload)) {
    return result;
  }
  for (const auto& robot : message.robots()) {
    MapGeoPoint point = PointFromLngLat(
        robot.lng_lat(),
        commsgs::TimeToNanoseconds(robot.header().stamp()),
        QString::fromStdString(robot.name()));
    point.heading_deg = robot.rotation_deg();
    if (robot.status() == "error") {
      point.color = QColor(255, 59, 48);
    } else if (robot.status() == "charging") {
      point.color = QColor(247, 168, 0);
    } else if (robot.status() == "running") {
      point.color = QColor(0, 102, 255);
    }
    result.points.push_back(point);
  }
  return result;
}

MapIngestResult IngestPoiMarkerArray(const std::string& payload) {
  MapIngestResult result;
  automsgs::msgs::strata_msgs::PoiMarkerArray message;
  if (!message.ParseFromString(payload)) {
    return result;
  }
  for (const auto& marker : message.markers()) {
    MapGeoPoint point = PointFromLngLat(
        marker.lng_lat(),
        commsgs::TimeToNanoseconds(marker.header().stamp()),
        QString::fromStdString(marker.name()));
    point.heading_deg = marker.rotation_deg();
    if (marker.selected()) {
      point.color = QColor(255, 180, 0);
    }
    result.points.push_back(point);
  }
  return result;
}

MapIngestResult IngestSemanticZoneArray(const std::string& payload) {
  MapIngestResult result;
  automsgs::msgs::strata_msgs::SemanticZoneArray message;
  if (!message.ParseFromString(payload)) {
    return result;
  }
  for (const auto& zone : message.zones()) {
    MapGeoPolygon polygon;
    polygon.fill_color = ColorFromRgba(zone.fill_color().r(), zone.fill_color().g(),
                                       zone.fill_color().b(), zone.fill_opacity());
    polygon.outline_color = ColorFromRgba(zone.outline_color().r(),
                                          zone.outline_color().g(),
                                          zone.outline_color().b());
    polygon.outline_width = zone.outline_width();
    for (const auto& vertex : zone.polygon()) {
      polygon.outer_ring.push_back(QPointF(vertex.x(), vertex.y()));
    }
    if (polygon.outer_ring.size() >= 3) {
      result.polygons.push_back(polygon);
    }
  }
  return result;
}

MapIngestResult IngestRoadGraph(const std::string& payload) {
  MapIngestResult result;
  automsgs::msgs::strata_msgs::RoadGraph message;
  if (!message.ParseFromString(payload)) {
    return result;
  }
  QHash<QString, QPointF> node_coords;
  for (const auto& node : message.nodes()) {
    node_coords.insert(QString::fromStdString(node.id()),
                       QPointF(node.coordinates().x(), node.coordinates().y()));
    MapGeoPoint point;
    point.longitude = node.coordinates().x();
    point.latitude = node.coordinates().y();
    point.label = QString::fromStdString(node.id());
    result.points.push_back(point);
  }
  for (const auto& edge : message.edges()) {
    const QPointF from = node_coords.value(QString::fromStdString(edge.from()));
    const QPointF to = node_coords.value(QString::fromStdString(edge.to()));
    if (from.isNull() || to.isNull()) {
      continue;
    }
    MapGeoLine line;
    line.lat_lon_points = {from, to};
    line.color = QColor(120, 120, 130);
    result.lines.push_back(line);
  }
  return result;
}

MapIngestResult IngestGeoJsonString(const std::string& payload, QString* error) {
  automsgs::msgs::std_msgs::String message;
  if (!message.ParseFromString(payload)) {
    if (error != nullptr) {
      *error = QStringLiteral("Failed to parse std_msgs/String");
    }
    return {};
  }
  return MapGeoJsonParser::Parse(QString::fromStdString(message.data()), error);
}

}  // namespace

bool MapMessageIngest::SupportsMessageType(const QString& message_type) {
  const QString normalized = NormalizeType(message_type);
  return normalized == QLatin1String("automsgs.msgs.sensor_msgs.NavSatFix") ||
         normalized == QLatin1String("automsgs.msgs.strata_msgs.RobotMarkerArray") ||
         normalized == QLatin1String("automsgs.msgs.strata_msgs.PoiMarkerArray") ||
         normalized == QLatin1String("automsgs.msgs.strata_msgs.SemanticZoneArray") ||
         normalized == QLatin1String("automsgs.msgs.strata_msgs.RoadGraph") ||
         normalized == QLatin1String("automsgs.msgs.std_msgs.String");
}

MapIngestResult MapMessageIngest::Ingest(const QString& message_type,
                                         const std::string& payload,
                                         QString* error) {
  const QString normalized = NormalizeType(message_type);
  if (normalized == QLatin1String("automsgs.msgs.sensor_msgs.NavSatFix")) {
    return IngestNavSatFix(payload);
  }
  if (normalized == QLatin1String("automsgs.msgs.strata_msgs.RobotMarkerArray")) {
    return IngestRobotMarkerArray(payload);
  }
  if (normalized == QLatin1String("automsgs.msgs.strata_msgs.PoiMarkerArray")) {
    return IngestPoiMarkerArray(payload);
  }
  if (normalized == QLatin1String("automsgs.msgs.strata_msgs.SemanticZoneArray")) {
    return IngestSemanticZoneArray(payload);
  }
  if (normalized == QLatin1String("automsgs.msgs.strata_msgs.RoadGraph")) {
    return IngestRoadGraph(payload);
  }
  if (normalized == QLatin1String("automsgs.msgs.std_msgs.String")) {
    return IngestGeoJsonString(payload, error);
  }
  if (error != nullptr) {
    *error = QStringLiteral("Unsupported message type");
  }
  return {};
}

}  // namespace map
}  // namespace autoviz

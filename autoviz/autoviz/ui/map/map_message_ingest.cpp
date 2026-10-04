/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/map/map_message_ingest.hpp"

#include <QHash>
#include <QPointF>

#include <QtMath>

#include <automsgs/msgs/sensor_msgs/nav_sat_fix.pb.h>
#include <automsgs/msgs/std_msgs/string.pb.h>

#include "autoviz/commsgs/message_type_utils.hpp"
#include "autoviz/commsgs/time_utils.hpp"

namespace autoviz {
namespace map {
namespace {

QString NormalizeType(const QString& message_type) {
  return QString::fromStdString(
      commsgs::NormalizeMessageType(message_type.toStdString()));
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
         normalized == QLatin1String("automsgs.msgs.std_msgs.String");
}

MapIngestResult MapMessageIngest::Ingest(const QString& message_type,
                                         const std::string& payload,
                                         QString* error) {
  const QString normalized = NormalizeType(message_type);
  if (normalized == QLatin1String("automsgs.msgs.sensor_msgs.NavSatFix")) {
    return IngestNavSatFix(payload);
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

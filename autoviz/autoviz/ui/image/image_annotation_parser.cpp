/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/image/image_annotation_parser.hpp"

#include <cmath>

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStringList>

#include <automsgs/msgs/vision_msgs/bounding_box2d.pb.h>
#include <automsgs/msgs/vision_msgs/bounding_box2d_array.pb.h>
#include <automsgs/msgs/vision_msgs/detection2d.pb.h>
#include <automsgs/msgs/vision_msgs/detection2d_array.pb.h>

namespace autoviz {
namespace image {
namespace {

QColor DefaultAnnotationColor() {
  return QColor(QStringLiteral("#00ff88"));
}

QColor ColorFromJson(const QJsonObject& object, const QColor& fallback) {
  if (object.isEmpty()) {
    return fallback;
  }
  return QColor::fromRgbF(object.value(QStringLiteral("r")).toDouble(0.0),
                          object.value(QStringLiteral("g")).toDouble(1.0),
                          object.value(QStringLiteral("b")).toDouble(0.0),
                          object.value(QStringLiteral("a")).toDouble(1.0));
}

QPointF PointFromJson(const QJsonObject& object) {
  return QPointF(object.value(QStringLiteral("x")).toDouble(),
                 object.value(QStringLiteral("y")).toDouble());
}

qint64 TimestampNsFromJson(const QJsonObject& object) {
  const QJsonObject timestamp = object.value(QStringLiteral("timestamp")).toObject();
  if (timestamp.isEmpty()) {
    return 0;
  }
  const qint64 sec = timestamp.value(QStringLiteral("sec")).toVariant().toLongLong();
  const qint64 nsec = timestamp.value(QStringLiteral("nsec")).toVariant().toLongLong();
  return sec * 1000000000LL + nsec;
}

ImageAnnotationPolyline PolylineFromPoints(const QVector<QPointF>& points,
                                           const QColor& color, float thickness,
                                           bool closed) {
  ImageAnnotationPolyline polyline;
  polyline.points = points;
  polyline.outline_color = color;
  polyline.thickness = thickness;
  polyline.closed = closed;
  return polyline;
}

QString FormatMetadata(const QJsonObject& object) {
  const QJsonValue metadata_value = object.value(QStringLiteral("metadata"));
  QStringList lines;
  if (metadata_value.isArray()) {
    for (const QJsonValue& entry : metadata_value.toArray()) {
      const QJsonObject pair = entry.toObject();
      const QString key = pair.value(QStringLiteral("key")).toString();
      const QString value = pair.value(QStringLiteral("value")).toString();
      if (key.isEmpty() && value.isEmpty()) {
        continue;
      }
      lines << (key.isEmpty() ? value : QStringLiteral("%1: %2").arg(key, value));
    }
  } else if (metadata_value.isObject()) {
    const QJsonObject meta = metadata_value.toObject();
    for (auto it = meta.begin(); it != meta.end(); ++it) {
      lines << QStringLiteral("%1: %2").arg(it.key(), it.value().toVariant().toString());
    }
  }
  return lines.join(QLatin1Char('\n'));
}

QString MergeTooltips(const QString& root_tip, const QString& local_tip) {
  if (root_tip.isEmpty()) {
    return local_tip;
  }
  if (local_tip.isEmpty()) {
    return root_tip;
  }
  return root_tip + QLatin1Char('\n') + local_tip;
}

ImageAnnotationLayer FromFoxgloveJsonObject(const QJsonObject& root) {
  ImageAnnotationLayer layer;
  layer.timestamp_ns = TimestampNsFromJson(root);
  const QString root_tip = FormatMetadata(root);

  for (const QJsonValue& value : root.value(QStringLiteral("circles")).toArray()) {
    const QJsonObject circle = value.toObject();
    const QPointF center = PointFromJson(circle.value(QStringLiteral("position")).toObject());
    const double diameter = circle.value(QStringLiteral("diameter")).toDouble(0.0);
    if (diameter <= 0.0) {
      continue;
    }
    const QColor color = ColorFromJson(
        circle.value(QStringLiteral("outline_color")).toObject(),
        DefaultAnnotationColor());
    const float thickness = static_cast<float>(
        circle.value(QStringLiteral("thickness")).toDouble(2.0));
    const double radius = diameter * 0.5;
    QVector<QPointF> ring;
    constexpr int kSegments = 32;
    ring.reserve(kSegments + 1);
    for (int i = 0; i <= kSegments; ++i) {
      const double theta = (2.0 * M_PI * i) / kSegments;
      ring.push_back(center + QPointF(std::cos(theta) * radius, std::sin(theta) * radius));
    }
    ImageAnnotationPolyline polyline =
        PolylineFromPoints(ring, color, thickness, true);
    polyline.tooltip = MergeTooltips(root_tip, FormatMetadata(circle));
    layer.polylines.push_back(std::move(polyline));
  }

  for (const QJsonValue& value : root.value(QStringLiteral("points")).toArray()) {
    const QJsonObject points_annotation = value.toObject();
    const QJsonArray points = points_annotation.value(QStringLiteral("points")).toArray();
    if (points.isEmpty()) {
      continue;
    }
    const QColor color = ColorFromJson(
        points_annotation.value(QStringLiteral("outline_color")).toObject(),
        DefaultAnnotationColor());
    const float thickness = static_cast<float>(
        points_annotation.value(QStringLiteral("thickness")).toDouble(2.0));
    const int type = points_annotation.value(QStringLiteral("type")).toInt(0);
    const QString tip =
        MergeTooltips(root_tip, FormatMetadata(points_annotation));
    QVector<QPointF> polyline_points;
    polyline_points.reserve(points.size());
    for (const QJsonValue& point_value : points) {
      polyline_points.push_back(PointFromJson(point_value.toObject()));
    }
    if (type == 0) {
      for (const QPointF& point : polyline_points) {
        ImageAnnotationPoint annotation_point;
        annotation_point.position = point;
        annotation_point.color = color;
        annotation_point.size = std::max(thickness, 4.0f);
        annotation_point.tooltip = tip;
        layer.points.push_back(annotation_point);
      }
      continue;
    }
    const bool closed = type == 3;
    ImageAnnotationPolyline polyline =
        PolylineFromPoints(polyline_points, color, thickness, closed);
    polyline.tooltip = tip;
    layer.polylines.push_back(std::move(polyline));
  }

  for (const QJsonValue& value : root.value(QStringLiteral("texts")).toArray()) {
    const QJsonObject text = value.toObject();
    ImageAnnotationText annotation_text;
    annotation_text.position = PointFromJson(text.value(QStringLiteral("position")).toObject());
    annotation_text.text = text.value(QStringLiteral("text")).toString();
    annotation_text.color = ColorFromJson(
        text.value(QStringLiteral("text_color")).toObject(), Qt::white);
    annotation_text.font_size = text.value(QStringLiteral("font_size")).toDouble(12.0);
    annotation_text.tooltip = MergeTooltips(root_tip, FormatMetadata(text));
    if (!annotation_text.text.isEmpty()) {
      layer.texts.push_back(annotation_text);
    }
  }

  return layer;
}

ImageAnnotationLayer FromFoxgloveJsonPayload(const std::string& payload) {
  const QJsonDocument document =
      QJsonDocument::fromJson(QByteArray::fromStdString(payload));
  if (!document.isObject()) {
    return {};
  }
  return FromFoxgloveJsonObject(document.object());
}

ImageAnnotationPolyline BoxPolyline(const automsgs::msgs::vision_msgs::BoundingBox2D& box,
                                    const QColor& color) {
  const double cx = box.center().position().x();
  const double cy = box.center().position().y();
  const double theta = box.center().theta();
  const double hx = box.size_x() * 0.5;
  const double hy = box.size_y() * 0.5;

  auto corner = [&](double lx, double ly) {
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    return QPointF(cx + c * lx - s * ly, cy + s * lx + c * ly);
  };

  ImageAnnotationPolyline polyline;
  polyline.outline_color = color;
  polyline.thickness = 2.0f;
  polyline.closed = true;
  polyline.points = {corner(-hx, -hy), corner(hx, -hy), corner(hx, hy),
                     corner(-hx, hy)};
  return polyline;
}

void AppendDetection2D(ImageAnnotationLayer& layer,
                       const automsgs::msgs::vision_msgs::Detection2D& det) {
  QStringList tip_parts;
  if (!det.id().empty()) {
    tip_parts << QString::fromStdString(det.id());
  }
  QString top_class;
  double top_score = 0.0;
  for (int i = 0; i < det.results_size(); ++i) {
    const auto& hyp = det.results(i).hypothesis();
    const QString cid = QString::fromStdString(hyp.class_id());
    tip_parts << QStringLiteral("%1:%2").arg(cid).arg(hyp.score(), 0, 'f', 2);
    if (i == 0 || hyp.score() > top_score) {
      top_score = hyp.score();
      top_class = cid;
    }
  }
  ImageAnnotationPolyline poly =
      BoxPolyline(det.bbox(), DefaultAnnotationColor());
  poly.tooltip = tip_parts.join(QStringLiteral(" | "));
  layer.polylines.push_back(poly);

  if (!top_class.isEmpty() || !det.id().empty()) {
    ImageAnnotationText text;
    text.position = QPointF(det.bbox().center().position().x(),
                            det.bbox().center().position().y() -
                                det.bbox().size_y() * 0.5 - 4.0);
    if (!top_class.isEmpty()) {
      text.text = QStringLiteral("%1 %.2f").arg(top_class).arg(top_score);
    } else {
      text.text = QString::fromStdString(det.id());
    }
    text.color = QColor(QStringLiteral("#e2e8f0"));
    text.font_size = 11.0;
    text.tooltip = poly.tooltip;
    layer.texts.push_back(text);
  }
}

qint64 HeaderTimestampNs(const automsgs::msgs::std_msgs::Header& header) {
  return static_cast<qint64>(header.stamp().sec()) * 1000000000LL +
         static_cast<qint64>(header.stamp().nanosec());
}

bool IsFoxgloveImageAnnotationsType(const std::string& message_type) {
  return message_type == "foxglove.ImageAnnotations" ||
         message_type.find("ImageAnnotations") != std::string::npos;
}

}  // namespace

ImageAnnotationLayer ImageAnnotationParser::fromPayload(
    const std::string& message_type, const std::string& payload) {
  if (IsFoxgloveImageAnnotationsType(message_type)) {
    if (!payload.empty() && payload.front() == '{') {
      return FromFoxgloveJsonPayload(payload);
    }
    return {};
  }

  ImageAnnotationLayer layer;
  if (message_type == "automsgs.msgs.vision_msgs.BoundingBox2DArray" ||
      message_type == "vision_msgs/BoundingBox2DArray") {
    automsgs::msgs::vision_msgs::BoundingBox2DArray message;
    if (!message.ParseFromString(payload)) {
      return layer;
    }
    layer.timestamp_ns = HeaderTimestampNs(message.header());
    for (int i = 0; i < message.boxes_size(); ++i) {
      layer.polylines.push_back(BoxPolyline(message.boxes(i), DefaultAnnotationColor()));
    }
    return layer;
  }

  if (message_type == "automsgs.msgs.vision_msgs.Detection2DArray" ||
      message_type == "vision_msgs/Detection2DArray") {
    automsgs::msgs::vision_msgs::Detection2DArray message;
    if (!message.ParseFromString(payload)) {
      return layer;
    }
    layer.timestamp_ns = HeaderTimestampNs(message.header());
    for (int i = 0; i < message.detections_size(); ++i) {
      AppendDetection2D(layer, message.detections(i));
    }
    return layer;
  }

  if (message_type == "automsgs.msgs.vision_msgs.Detection2D" ||
      message_type == "vision_msgs/Detection2D") {
    automsgs::msgs::vision_msgs::Detection2D message;
    if (!message.ParseFromString(payload)) {
      return layer;
    }
    layer.timestamp_ns = HeaderTimestampNs(message.header());
    AppendDetection2D(layer, message);
    return layer;
  }

  if (message_type == "automsgs.msgs.vision_msgs.BoundingBox2D" ||
      message_type == "vision_msgs/BoundingBox2D") {
    automsgs::msgs::vision_msgs::BoundingBox2D message;
    if (!message.ParseFromString(payload)) {
      return layer;
    }
    layer.polylines.push_back(BoxPolyline(message, DefaultAnnotationColor()));
    return layer;
  }

  return layer;
}

}  // namespace image
}  // namespace autoviz

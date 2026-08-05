/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/image/image_marker_projection.hpp"

#include <QMatrix4x4>
#include <QQuaternion>

#include "autoviz/commsgs/time_utils.hpp"

namespace autoviz {
namespace image {
namespace {

QVector3D ToQVector3(const automsgs::msgs::geometry_msgs::Point& point) {
  return QVector3D(static_cast<float>(point.x()), static_cast<float>(point.y()),
                     static_cast<float>(point.z()));
}

QMatrix4x4 TransformToMatrix(
    const automsgs::msgs::geometry_msgs::Transform& transform) {
  QMatrix4x4 matrix;
  matrix.setToIdentity();
  matrix.translate(static_cast<float>(transform.translation().x()),
                   static_cast<float>(transform.translation().y()),
                   static_cast<float>(transform.translation().z()));
  matrix.rotate(
      QQuaternion(static_cast<float>(transform.rotation().w()),
                  static_cast<float>(transform.rotation().x()),
                  static_cast<float>(transform.rotation().y()),
                  static_cast<float>(transform.rotation().z())));
  return matrix;
}

void AppendProjectedPolyline(const std::vector<QPointF>& points,
                             ImageAnnotationLayer* layer,
                             const QColor& color) {
  if (layer == nullptr || points.size() < 2) {
    return;
  }
  ImageAnnotationPolyline polyline;
  polyline.outline_color = color;
  polyline.thickness = 2.0f;
  polyline.closed = false;
  polyline.points.reserve(static_cast<int>(points.size()));
  for (const QPointF& point : points) {
    polyline.points.push_back(point);
  }
  layer->polylines.push_back(std::move(polyline));
}

}  // namespace

ImageAnnotationLayer projectMarkerToLayer(
    const automsgs::msgs::visualization_msgs::Marker& marker,
    const CameraIntrinsics& intrinsics, const QMatrix4x4& fixed_to_optical,
    const std::string& fixed_frame, autoviz::transform::Buffer* tf_buffer) {
  ImageAnnotationLayer layer;
  if (!intrinsics.valid || tf_buffer == nullptr) {
    return layer;
  }

  QMatrix4x4 marker_to_fixed;
  marker_to_fixed.setToIdentity();
  try {
    const auto zero_time = autoviz::commsgs::ZeroTime();
    const auto transform = tf_buffer->lookupTransform(
        fixed_frame, marker.header().frame_id(), zero_time);
    marker_to_fixed = TransformToMatrix(transform.transform());
  } catch (...) {
    marker_to_fixed.setToIdentity();
  }
  QMatrix4x4 marker_pose;
  marker_pose.setToIdentity();
  marker_pose.translate(ToQVector3(marker.pose().position()));
  marker_pose.rotate(QQuaternion(static_cast<float>(marker.pose().orientation().w()),
                                 static_cast<float>(marker.pose().orientation().x()),
                                 static_cast<float>(marker.pose().orientation().y()),
                                 static_cast<float>(marker.pose().orientation().z())));
  const QMatrix4x4 marker_to_fixed_pose = marker_to_fixed * marker_pose;

  const QColor color(static_cast<int>(marker.color().r() * 255.0),
                       static_cast<int>(marker.color().g() * 255.0),
                       static_cast<int>(marker.color().b() * 255.0));

  auto projectPoint = [&](const QVector3D& marker_point) -> std::optional<QPointF> {
    const QVector3D fixed = marker_to_fixed_pose.map(marker_point);
    return projectFixedPointToPixel(intrinsics, fixed_to_optical, fixed);
  };

  std::vector<QPointF> projected;
  projected.reserve(marker.points_size());
  for (int i = 0; i < marker.points_size(); ++i) {
    if (const auto pixel = projectPoint(ToQVector3(marker.points(i)))) {
      projected.push_back(*pixel);
    }
  }

  if (marker.type() == 4 || marker.type() == 5 || marker.type() == 6) {
    AppendProjectedPolyline(projected, &layer, color);
    return layer;
  }

  if (marker.type() == 1 || marker.type() == 2 || marker.type() == 3) {
    const float sx = static_cast<float>(marker.scale().x()) * 0.5f;
    const float sy = static_cast<float>(marker.scale().y()) * 0.5f;
    const float sz = static_cast<float>(marker.scale().z()) * 0.5f;
    const std::vector<QVector3D> corners = {
        {-sx, -sy, -sz}, {sx, -sy, -sz}, {sx, sy, -sz}, {-sx, sy, -sz},
        {-sx, -sy, sz},  {sx, -sy, sz},  {sx, sy, sz},  {-sx, sy, sz}};
    std::vector<QPointF> box_front;
    for (int i = 0; i < 4; ++i) {
      if (const auto pixel = projectPoint(corners[i])) {
        box_front.push_back(*pixel);
      }
    }
    box_front.push_back(box_front.front());
    AppendProjectedPolyline(box_front, &layer, color);
    return layer;
  }

  for (const QPointF& point : projected) {
    ImageAnnotationPoint annotation_point;
    annotation_point.position = point;
    annotation_point.color = color;
    annotation_point.size = 4.0f;
    layer.points.push_back(annotation_point);
  }
  return layer;
}

}  // namespace image
}  // namespace autoviz

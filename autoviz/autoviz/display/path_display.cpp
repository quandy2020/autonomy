/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/path_display.hpp"

#include <QColor>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/ogre_overlay_draw.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {
namespace {

struct LocalPathPose {
  QVector3D position;
  QQuaternion orientation;
};

LocalPathPose makePathPose(
    const automsgs::msgs::geometry_msgs::PoseStamped& pose_stamped,
    bool have_tf,
    const automsgs::msgs::geometry_msgs::TransformStamped& tf,
    const QVector3D& offset) {
  QVector3D p(static_cast<float>(pose_stamped.pose().position().x()),
              static_cast<float>(pose_stamped.pose().position().y()),
              static_cast<float>(pose_stamped.pose().position().z()));
  if (have_tf) {
    p = transformPoint(tf, p);
  }
  p += offset;
  const auto& q = pose_stamped.pose().orientation();
  QQuaternion orientation(static_cast<float>(q.w()), static_cast<float>(q.x()),
                          static_cast<float>(q.y()), static_cast<float>(q.z()));
  if (have_tf) {
    const auto& rq = tf.transform().rotation();
    const QQuaternion frame_q(static_cast<float>(rq.w()),
                            static_cast<float>(rq.x()),
                            static_cast<float>(rq.y()),
                            static_cast<float>(rq.z()));
    orientation = frame_q * orientation;
  }
  return {p, orientation.normalized()};
}

QVector3D rotateByQuaternion(const QQuaternion& q, const QVector3D& v) {
  return q.rotatedVector(v);
}

}  // namespace

PathDisplay::PathDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::nav_msgs::Path>(
          "Path", std::move(channel),
          "automsgs.msgs.nav_msgs.Path") {
  setProperties({});
  resizeBuffers(1);
}

std::vector<common::DisplayPropertySpec> PathDisplay::propertySpecs() const {
  return {{"line_style", "Line Style", "Lines", {"Lines", "Billboards"}},
          {"line_width", "Line Width", "0.03"},
          {"color", "Color", "25;255;0", {}, common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "1.0"},
          {"buffer_length", "Buffer Length", "1"},
          {"offset", "Offset", "0;0;0"},
          {"pose_style", "Pose Style", "None", {"None", "Axes", "Arrows"}},
          {"pose_axes_length", "Length", "0.3"},
          {"pose_axes_radius", "Radius", "0.03"},
          {"pose_color", "Pose Color", "255;85;255",
           {}, common::DisplayPropertyKind::kColor},
          {"pose_arrow_shaft_length", "Shaft Length", "0.1"},
          {"pose_arrow_head_length", "Head Length", "0.2"},
          {"pose_arrow_shaft_diameter", "Shaft Diameter", "0.1"},
          {"pose_arrow_head_diameter", "Head Diameter", "0.3"}};
}

void PathDisplay::resizeBuffers(std::size_t buffer_length) {
  if (buffer_length < 1) {
    buffer_length = 1;
  }
  path_buffers_.resize(buffer_length);
}

bool PathDisplay::useBillboardLineStyle() const {
  const std::string style = propertyValue("line_style", "Lines");
  if (style == "Billboards") {
    return true;
  }
  if (style == "Lines") {
    return false;
  }
  // Legacy configs without line_style: positive line_width implied billboards.
  return common::ParseFloatProperty(propertyValue("line_width", "0.03"), 0.03f) >
         0.f;
}

void PathDisplay::processMessage(
    const automsgs::msgs::nav_msgs::Path& message) {
  if (context_ == nullptr) {
    return;
  }

  const int buffer_length =
      std::max(1, common::ParseIntProperty(propertyValue("buffer_length", "1"), 1));
  if (path_buffers_.size() != static_cast<std::size_t>(buffer_length)) {
    resizeBuffers(static_cast<std::size_t>(buffer_length));
  }

  const QVector3D offset = common::ParseVector3Property(
      propertyValue("offset", "0;0;0"), QVector3D());

  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = message.header().frame_id().empty()
                                ? context_->fixed_frame
                                : message.header().frame_id();
  automsgs::msgs::geometry_msgs::TransformStamped tf;
  bool have_tf = false;
  if (frame != context_->fixed_frame) {
    try {
      tf = context_->tf_buffer->lookupTransform(context_->fixed_frame, frame,
                                                zero_time);
      have_tf = true;
    } catch (...) {
      return;
    }
  }

  const std::size_t buffer_index =
      messages_received_ % static_cast<std::size_t>(buffer_length);
  ++messages_received_;

  PathSnapshot& snapshot = path_buffers_[buffer_index];
  snapshot.clear_poses();
  snapshot.poses().reserve(static_cast<std::size_t>(message.poses_size()));
  for (const auto& pose_stamped : message.poses()) {
    const LocalPathPose local = makePathPose(pose_stamped, have_tf, tf, offset);
    *snapshot.mutable_poses()->Add() = {local.position(), local.orientation};
  }

  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void PathDisplay::drawPathPolyline(
    rendering::SceneOverlay& scene, const std::string& suffix,
    const std::vector<PathPose>& poses, const QColor& color,
    float line_width) const {
  if (poses.size() < 2) {
    return;
  }
  std::vector<QVector3D> points;
  points.reserve(poses.size());
  for (const auto& pose : poses) {
    points.push_back(pose.position());
  }
  const std::string draw_name = name() + suffix;
  if (useBillboardLineStyle()) {
    drawBillboardStripOgreOrGl(context_, scene, draw_name, points, color,
                               line_width);
    return;
  }
  std::vector<LineSegment3D> segments;
  segments.reserve(points.size() - 1);
  for (std::size_t i = 1; i < points.size(); ++i) {
    segments.push_back({points[i - 1], points[i], color});
  }
  drawLineSegmentsOgreOrGl(context_, scene, draw_name, segments);
}

void PathDisplay::drawPoseMarkers(
    rendering::SceneOverlay& scene, const std::string& suffix,
    const std::vector<PathPose>& poses) const {
  const std::string pose_style = propertyValue("pose_style", "None");
  if (pose_style == "None" || poses.empty()) {
    return;
  }

  if (pose_style == "Axes") {
    const float length = common::ParseFloatProperty(
        propertyValue("pose_axes_length", "0.3"), 0.3f);
    (void)common::ParseFloatProperty(propertyValue("pose_axes_radius", "0.03"),
                                     0.03f);
    std::vector<LineSegment3D> segments;
    segments.reserve(poses.size() * 3);
    for (const auto& pose : poses) {
      const QVector3D x_end =
          pose.position + rotateByQuaternion(pose.orientation(), QVector3D(length, 0.f, 0.f));
      const QVector3D y_end =
          pose.position + rotateByQuaternion(pose.orientation(), QVector3D(0.f, length, 0.f));
      const QVector3D z_end =
          pose.position + rotateByQuaternion(pose.orientation(), QVector3D(0.f, 0.f, length));
      segments.push_back({pose.position(), x_end, QColor(220, 60, 60)});
      segments.push_back({pose.position(), y_end, QColor(60, 220, 60)});
      segments.push_back({pose.position(), z_end, QColor(60, 120, 220)});
    }
    drawLineSegmentsOgreOrGl(context_, scene, name() + suffix + "/axes",
                             segments);
    return;
  }

  if (pose_style == "Arrows") {
    const QColor arrow_color = common::ParseColorProperty(
        propertyValue("pose_color", "255;85;255"), QColor(255, 85, 255));
    const float shaft_length = common::ParseFloatProperty(
        propertyValue("pose_arrow_shaft_length", "0.1"), 0.1f);
    const float head_length = common::ParseFloatProperty(
        propertyValue("pose_arrow_head_length", "0.2"), 0.2f);
    const float shaft_diameter = common::ParseFloatProperty(
        propertyValue("pose_arrow_shaft_diameter", "0.1"), 0.1f);
    const float head_diameter = common::ParseFloatProperty(
        propertyValue("pose_arrow_head_diameter", "0.3"), 0.3f);
    const float total_length = shaft_length + head_length;
    const float head_fraction =
        total_length > 0.f ? head_length / total_length : 0.2f;
    for (std::size_t i = 0; i < poses.size(); ++i) {
      const auto& pose = poses[i];
      const QVector3D direction =
          rotateByQuaternion(pose.orientation(), QVector3D(total_length, 0.f, 0.f));
      drawArrowOgreOrGl(context_, scene,
                        name() + suffix + "/arrow/" + std::to_string(i),
                        pose.position(), pose.position + direction, arrow_color,
                        head_fraction, shaft_diameter, head_diameter);
    }
  }
}

void PathDisplay::onDraw(rendering::SceneOverlay& scene) {
  QColor color =
      common::ParseColorProperty(propertyValue("color", "25;255;0"),
                                 QColor(25, 255, 0));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);
  color.setAlphaF(alpha);
  const float line_width =
      common::ParseFloatProperty(propertyValue("line_width", "0.03"), 0.03f);

  for (std::size_t i = 0; i < path_buffers_.size(); ++i) {
    const std::string suffix = "/buffer/" + std::to_string(i);
    drawPathPolyline(scene, suffix, path_buffers_[i].poses, color, line_width);
    drawPoseMarkers(scene, suffix, path_buffers_[i].poses);
  }
}

}  // namespace display
}  // namespace autoviz

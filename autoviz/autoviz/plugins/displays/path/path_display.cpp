/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/path/path_display.hpp"

#include <QColor>
#include <QTime>

#include <algorithm>
#include <cmath>
#include <limits>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/common/display_tf_utils.hpp"
#include "autoviz/plugins/displays/common/ogre_mesh_draw.hpp"
#include "autoviz/plugins/displays/common/ogre_overlay_draw.hpp"
#include "autoviz/plugins/displays/common/transform_utils.hpp"
#include "autoviz/commsgs/time_utils.hpp"

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

constexpr float kPathSurfaceLiftZ = 0.05f;

QVector3D LiftPathPoint(const QVector3D& p) {
  return QVector3D(p.x(), p.y(), p.z() + kPathSurfaceLiftZ);
}

struct PolySample {
  std::vector<QVector3D> points;
  std::vector<float> cum_len;
  float total_len = 0.f;
};

PolySample BuildPolyline(const std::vector<QVector3D>& raw) {
  PolySample sample;
  if (raw.size() < 2) {
    return sample;
  }
  sample.points = raw;
  sample.cum_len.resize(raw.size(), 0.f);
  for (std::size_t i = 1; i < raw.size(); ++i) {
    sample.cum_len[i] =
        sample.cum_len[i - 1] + (raw[i] - raw[i - 1]).length();
  }
  sample.total_len = sample.cum_len.back();
  return sample;
}

bool SampleAt(const PolySample& poly, float s, QVector3D* pos,
              QVector3D* tangent) {
  if (poly.points.size() < 2 || poly.total_len <= 1e-5f || pos == nullptr) {
    return false;
  }
  s = std::clamp(s, 0.f, poly.total_len);
  std::size_t i = 1;
  while (i + 1 < poly.cum_len.size() && poly.cum_len[i] < s) {
    ++i;
  }
  const float s0 = poly.cum_len[i - 1];
  const float s1 = poly.cum_len[i];
  const float seg = std::max(1e-5f, s1 - s0);
  const float u = (s - s0) / seg;
  *pos = poly.points[i - 1] * (1.f - u) + poly.points[i] * u;
  if (tangent != nullptr) {
    *tangent = (poly.points[i] - poly.points[i - 1]).normalized();
  }
  return true;
}

}  // namespace

PathDisplay::PathDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::nav_msgs::Path>(
          "Path", std::move(channel), "automsgs.msgs.nav_msgs.Path") {
  setProperties({});
  resizeBuffers(1);
}

std::vector<common::DisplayPropertySpec> PathDisplay::propertySpecs() const {
  return {{"line_style", "Line Style", "Lines", {"Lines", "Billboards"}},
          {"path_pattern", "Path Pattern", "Solid",
           {"Solid", "Dashed", "FlowDashed"}},
          {"line_width", "Line Width", "0.03"},
          {"color", "Color", "25;255;0", {}, common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "1.0"},
          {"dash_length", "Dash Length (m)", "0.28"},
          {"gap_length", "Gap Length (m)", "0.18"},
          {"flow_speed", "Flow Speed (m/s)", "0.9"},
          {"show_flow_arrows", "Show Flow Arrows", "true", {"true", "false"}},
          {"flow_arrow_spacing", "Flow Arrow Spacing (m)", "0.55"},
          {"flow_arrow_size", "Flow Arrow Size (m)", "0.12"},
          {"prune_behind_robot", "Prune Behind Robot", "true",
           {"true", "false"}},
          {"hide_when_arrived", "Hide When Arrived", "true",
           {"true", "false"}},
          {"arrive_distance", "Arrive Distance (m)", "0.35"},
          {"robot_frame", "Robot Frame", "base_link"},
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
  return common::ParseFloatProperty(propertyValue("line_width", "0.03"), 0.03f) >
         0.f;
}

std::string PathDisplay::pathPattern() const {
  return propertyValue("path_pattern", "Solid");
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
      setStatusWarn("TF missing: " + frame + " -> " + context_->fixed_frame);
      return;
    }
  }

  const std::size_t buffer_index =
      messages_received_ % static_cast<std::size_t>(buffer_length);
  ++messages_received_;

  PathSnapshot& snapshot = path_buffers_[buffer_index];
  snapshot.poses.clear();
  snapshot.poses.reserve(static_cast<std::size_t>(message.poses_size()));
  for (const auto& pose_stamped : message.poses()) {
    const LocalPathPose local = makePathPose(pose_stamped, have_tf, tf, offset);
    snapshot.poses.push_back({local.position, local.orientation});
  }

  if (context_->request_redraw) {
    context_->request_redraw();
  }
  setStatusOk();
}

void PathDisplay::clearReceivedData() {
  for (auto& buffer : path_buffers_) {
    buffer.poses.clear();
  }
  messages_received_ = 0;
}

void PathDisplay::onUpdate() {
  ChannelDisplay::onUpdate();
  // Animate FlowDashed, and re-evaluate prune / arrival hide against TF.
  if (context_ != nullptr && context_->request_redraw) {
    for (const auto& buffer : path_buffers_) {
      if (buffer.poses.size() >= 2) {
        context_->request_redraw();
        break;
      }
    }
  }
}

bool PathDisplay::lookupRobotPosition(QVector3D* out) const {
  if (out == nullptr || context_ == nullptr || context_->tf_buffer == nullptr) {
    return false;
  }
  const std::string preferred = propertyValue("robot_frame", "base_link");
  const std::string candidates[] = {preferred, "base_link", "base_footprint"};
  const auto zero_time = autoviz::commsgs::ZeroTime();
  for (const auto& frame : candidates) {
    if (frame.empty()) {
      continue;
    }
    try {
      const auto tf = context_->tf_buffer->lookupTransform(
          context_->fixed_frame, frame, zero_time);
      *out = QVector3D(static_cast<float>(tf.transform().translation().x()),
                       static_cast<float>(tf.transform().translation().y()),
                       static_cast<float>(tf.transform().translation().z()));
      return true;
    } catch (...) {
    }
  }
  return false;
}

bool PathDisplay::robotArrivedAtPathEnd(
    const std::vector<PathPose>& poses) const {
  if (poses.empty()) {
    return true;
  }
  QVector3D robot;
  if (!lookupRobotPosition(&robot)) {
    return false;
  }
  const float arrive = std::max(
      0.05f, common::ParseFloatProperty(
                 propertyValue("arrive_distance", "0.35"), 0.35f));
  const QVector3D& end = poses.back().position;
  const float dx = robot.x() - end.x();
  const float dy = robot.y() - end.y();
  return (dx * dx + dy * dy) <= arrive * arrive;
}

std::vector<PathDisplay::PathPose> PathDisplay::prunePosesBehindRobot(
    const std::vector<PathPose>& poses) const {
  if (poses.size() < 2) {
    return poses;
  }
  QVector3D robot;
  if (!lookupRobotPosition(&robot)) {
    return poses;
  }

  std::size_t closest = 0;
  float best = std::numeric_limits<float>::max();
  for (std::size_t i = 0; i < poses.size(); ++i) {
    const float dx = poses[i].position.x() - robot.x();
    const float dy = poses[i].position.y() - robot.y();
    const float d2 = dx * dx + dy * dy;
    if (d2 < best) {
      best = d2;
      closest = i;
    }
  }
  // Keep a tiny look-back so the arrow under the robot is not clipped.
  const std::size_t start = closest > 0 ? closest - 1 : 0;
  return std::vector<PathPose>(poses.begin() + static_cast<std::ptrdiff_t>(start),
                               poses.end());
}

void PathDisplay::clearPathGeometry(rendering::SceneOverlay& scene,
                                    const std::string& suffix) const {
  drawBillboardStripOgreOrGl(context_, scene, name() + suffix, {},
                             QColor(0, 0, 0, 0), 0.01f);
  drawBillboardStripOgreOrGl(context_, scene, name() + suffix + "/underlay", {},
                             QColor(0, 0, 0, 0), 0.01f);
  drawLineSegmentsOgreOrGl(context_, scene, name() + suffix, {});
  drawLineSegmentsOgreOrGl(context_, scene, name() + suffix + "/axes", {});
  constexpr std::size_t kMaxDashes = 96;
  for (std::size_t i = 0; i < kMaxDashes; ++i) {
    const std::string dash_name =
        name() + suffix + "/dash/" + std::to_string(i);
    drawBillboardStripOgreOrGl(context_, scene, dash_name, {},
                               QColor(0, 0, 0, 0), 0.01f);
    drawLineSegmentsOgreOrGl(context_, scene, dash_name, {});
  }
  constexpr std::size_t kMaxArrows = 80;
  for (std::size_t i = 0; i < kMaxArrows; ++i) {
    drawMeshesOgreOrGl(context_, scene,
                       name() + suffix + "/flow_arrow/" + std::to_string(i),
                       {});
    drawMeshesOgreOrGl(context_, scene,
                       name() + suffix + "/arrow/" + std::to_string(i), {});
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
    points.push_back(LiftPathPoint(pose.position));
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

void PathDisplay::drawDashedPath(
    rendering::SceneOverlay& scene, const std::string& suffix,
    const std::vector<PathPose>& poses, const QColor& color, float line_width,
    bool animate) const {
  if (poses.size() < 2) {
    return;
  }
  std::vector<QVector3D> raw;
  raw.reserve(poses.size());
  for (const auto& pose : poses) {
    raw.push_back(LiftPathPoint(pose.position));
  }
  const PolySample poly = BuildPolyline(raw);
  if (poly.total_len < 1e-3f) {
    return;
  }

  const float dash_len = std::max(
      0.05f,
      common::ParseFloatProperty(propertyValue("dash_length", "0.28"), 0.28f));
  const float gap_len = std::max(
      0.02f,
      common::ParseFloatProperty(propertyValue("gap_length", "0.18"), 0.18f));
  const float period = dash_len + gap_len;
  float phase = 0.f;
  if (animate) {
    const float speed = common::ParseFloatProperty(
        propertyValue("flow_speed", "0.9"), 0.9f);
    const float t_sec =
        static_cast<float>(QTime::currentTime().msecsSinceStartOfDay()) *
        0.001f;
    phase = std::fmod(t_sec * speed, period);
    if (phase < 0.f) {
      phase += period;
    }
  }

  // Sample dashes as short polylines (billboard) or segments.
  constexpr float kSampleStep = 0.05f;
  constexpr std::size_t kMaxDashes = 96;
  std::size_t dash_id = 0;
  float s = -phase;
  while (s < poly.total_len && dash_id < kMaxDashes) {
    const float dash_start = std::max(0.f, s);
    const float dash_end = std::min(poly.total_len, s + dash_len);
    s += period;
    if (dash_end - dash_start < 0.02f) {
      continue;
    }

    std::vector<QVector3D> dash_pts;
    for (float u = dash_start; u <= dash_end + 1e-4f; u += kSampleStep) {
      QVector3D p;
      if (SampleAt(poly, std::min(u, dash_end), &p, nullptr)) {
        dash_pts.push_back(p);
      }
    }
    QVector3D end_p;
    if (SampleAt(poly, dash_end, &end_p, nullptr)) {
      if (dash_pts.empty() || (dash_pts.back() - end_p).length() > 1e-4f) {
        dash_pts.push_back(end_p);
      }
    }
    if (dash_pts.size() < 2) {
      continue;
    }

    const std::string dash_name =
        name() + suffix + "/dash/" + std::to_string(dash_id++);
    if (useBillboardLineStyle()) {
      drawBillboardStripOgreOrGl(context_, scene, dash_name, dash_pts, color,
                                 line_width);
    } else {
      std::vector<LineSegment3D> segs;
      segs.reserve(dash_pts.size() - 1);
      for (std::size_t i = 1; i < dash_pts.size(); ++i) {
        segs.push_back({dash_pts[i - 1], dash_pts[i], color});
      }
      drawLineSegmentsOgreOrGl(context_, scene, dash_name, segs);
    }
  }

  // Clear leftover dash slots from a previously longer path.
  for (std::size_t i = dash_id; i < kMaxDashes; ++i) {
    const std::string dash_name =
        name() + suffix + "/dash/" + std::to_string(i);
    drawBillboardStripOgreOrGl(context_, scene, dash_name, {},
                               QColor(0, 0, 0, 0), 0.01f);
    drawLineSegmentsOgreOrGl(context_, scene, dash_name, {});
  }
}

void PathDisplay::drawFlowChevrons(
    rendering::SceneOverlay& scene, const std::string& suffix,
    const std::vector<PathPose>& poses, const QColor& color) const {
  if (poses.size() < 2) {
    return;
  }
  std::vector<QVector3D> raw;
  raw.reserve(poses.size());
  for (const auto& pose : poses) {
    raw.push_back(LiftPathPoint(pose.position));
  }
  const PolySample poly = BuildPolyline(raw);
  if (poly.total_len < 0.1f) {
    return;
  }

  const float spacing = std::max(
      0.15f, common::ParseFloatProperty(
                 propertyValue("flow_arrow_spacing", "0.55"), 0.55f));
  const float size = std::max(
      0.04f,
      common::ParseFloatProperty(propertyValue("flow_arrow_size", "0.12"), 0.12f));
  const float line_width = std::max(
      0.02f,
      common::ParseFloatProperty(propertyValue("line_width", "0.03"), 0.03f));
  // Compact flow ticks: short body, wide head matched to path thickness.
  const float shaft_d = std::max(0.04f, line_width * 1.25f);
  const float head_d = std::max(0.09f, line_width * 2.6f);
  const float speed =
      common::ParseFloatProperty(propertyValue("flow_speed", "0.9"), 0.9f);
  const float t_sec =
      static_cast<float>(QTime::currentTime().msecsSinceStartOfDay()) * 0.001f;
  float phase = std::fmod(t_sec * speed, spacing);
  if (phase < 0.f) {
    phase += spacing;
  }

  QColor c = color;
  c.setAlphaF(std::min(1.f, color.alphaF() + 0.08f));

  constexpr std::size_t kMaxArrows = 80;
  std::size_t arrow_i = 0;
  for (float s = phase; s <= poly.total_len && arrow_i < kMaxArrows;
       s += spacing, ++arrow_i) {
    QVector3D p;
    QVector3D tangent;
    if (!SampleAt(poly, s, &p, &tangent) || tangent.length() < 1e-4f) {
      continue;
    }
    tangent.normalize();
    // Sit arrow slightly above dashes so thickness reads clearly.
    p.setZ(p.z() + 0.02f);
    const QVector3D start = p - tangent * (size * 0.20f);
    const QVector3D tip = p + tangent * (size * 0.80f);
    drawArrowOgreOrGl(context_, scene,
                      name() + suffix + "/flow_arrow/" + std::to_string(arrow_i),
                      start, tip, c, 0.62f, shaft_d, head_d);
  }
  // Clear leftover arrows from a longer previous path.
  for (std::size_t i = arrow_i; i < kMaxArrows; ++i) {
    drawMeshesOgreOrGl(context_, scene,
                       name() + suffix + "/flow_arrow/" + std::to_string(i),
                       {});
  }
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
      const QVector3D origin = LiftPathPoint(pose.position);
      const QVector3D x_end =
          origin + rotateByQuaternion(pose.orientation, QVector3D(length, 0.f, 0.f));
      const QVector3D y_end =
          origin + rotateByQuaternion(pose.orientation, QVector3D(0.f, length, 0.f));
      const QVector3D z_end =
          origin + rotateByQuaternion(pose.orientation, QVector3D(0.f, 0.f, length));
      segments.push_back({origin, x_end, QColor(220, 60, 60)});
      segments.push_back({origin, y_end, QColor(60, 220, 60)});
      segments.push_back({origin, z_end, QColor(60, 120, 220)});
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
      const QVector3D origin = LiftPathPoint(pose.position);
      const QVector3D direction =
          rotateByQuaternion(pose.orientation, QVector3D(total_length, 0.f, 0.f));
      drawArrowOgreOrGl(context_, scene,
                        name() + suffix + "/arrow/" + std::to_string(i),
                        origin, origin + direction, arrow_color,
                        head_fraction, shaft_diameter, head_diameter);
    }
  }
}

void PathDisplay::onDraw(rendering::SceneOverlay& scene) {
  QColor color = common::ParseColorProperty(propertyValue("color", "25;255;0"),
                                            QColor(25, 255, 0));
  const float alpha = std::clamp(
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f), 0.f, 1.f);
  color.setAlphaF(alpha);
  const float line_width =
      common::ParseFloatProperty(propertyValue("line_width", "0.03"), 0.03f);
  const std::string pattern = pathPattern();
  const bool prune = common::ParseBoolProperty(
      propertyValue("prune_behind_robot", "true"), true);
  const bool hide_arrived = common::ParseBoolProperty(
      propertyValue("hide_when_arrived", "true"), true);

  for (std::size_t i = 0; i < path_buffers_.size(); ++i) {
    const std::string suffix = "/buffer/" + std::to_string(i);
    const auto& raw_poses = path_buffers_[i].poses;
    if (raw_poses.size() < 2 ||
        (hide_arrived && robotArrivedAtPathEnd(raw_poses))) {
      clearPathGeometry(scene, suffix);
      continue;
    }

    const std::vector<PathPose> poses =
        prune ? prunePosesBehindRobot(raw_poses) : raw_poses;
    if (poses.size() < 2) {
      clearPathGeometry(scene, suffix);
      continue;
    }

    if (pattern == "Dashed") {
      drawDashedPath(scene, suffix, poses, color, line_width, false);
      drawBillboardStripOgreOrGl(context_, scene, name() + suffix, {},
                                 QColor(0, 0, 0, 0), 0.01f);
      drawBillboardStripOgreOrGl(context_, scene, name() + suffix + "/underlay",
                                 {}, QColor(0, 0, 0, 0), 0.01f);
    } else if (pattern == "FlowDashed") {
      QColor underlay = color;
      underlay.setAlphaF(std::clamp(alpha * 0.28f, 0.08f, 0.40f));
      drawPathPolyline(scene, suffix + "/underlay", poses, underlay,
                       std::max(0.04f, line_width * 1.65f));
      drawDashedPath(scene, suffix, poses, color, line_width, true);
      drawBillboardStripOgreOrGl(context_, scene, name() + suffix, {},
                                 QColor(0, 0, 0, 0), 0.01f);
      if (common::ParseBoolProperty(
              propertyValue("show_flow_arrows", "true"), true)) {
        drawFlowChevrons(scene, suffix, poses, color);
      }
    } else {
      drawPathPolyline(scene, suffix, poses, color, line_width);
      drawBillboardStripOgreOrGl(context_, scene, name() + suffix + "/underlay",
                                 {}, QColor(0, 0, 0, 0), 0.01f);
    }
    drawPoseMarkers(scene, suffix, poses);
  }
}

}  // namespace display
}  // namespace autoviz

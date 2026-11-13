/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/pose/target_visualizer_display.hpp"

#include <algorithm>
#include <utility>
#include <vector>

#include <cmath>

#include <QMatrix4x4>
#include <QTime>
#include <QtMath>

#include "autoviz/plugins/displays/common/display_tf_utils.hpp"
#include "autoviz/plugins/displays/common/ogre_mesh_draw.hpp"
#include "autoviz/plugins/displays/common/ogre_overlay_draw.hpp"
#include "autoviz/plugins/displays/common/primitive_mesh.hpp"
#include "autoviz/plugins/tools/goal_pose/goal_common.hpp"

namespace autoviz {
namespace display {
namespace {

std::vector<QVector3D> MakeRingPolyline(const QVector3D& center, float radius,
                                        float z_lift, int segments) {
  std::vector<QVector3D> points;
  points.reserve(static_cast<std::size_t>(segments) + 1);
  for (int s = 0; s <= segments; ++s) {
    const float a =
        static_cast<float>(2.0 * M_PI * s / static_cast<float>(segments));
    points.emplace_back(center.x() + radius * qCos(a),
                        center.y() + radius * qSin(a), center.z() + z_lift);
  }
  return points;
}

float Smoothstep(float t) {
  t = std::clamp(t, 0.f, 1.f);
  return t * t * (3.f - 2.f * t);
}

QColor LerpColor(const QColor& a, const QColor& b, float t) {
  t = std::clamp(t, 0.f, 1.f);
  QColor out;
  out.setRedF(a.redF() + (b.redF() - a.redF()) * t);
  out.setGreenF(a.greenF() + (b.greenF() - a.greenF()) * t);
  out.setBlueF(a.blueF() + (b.blueF() - a.blueF()) * t);
  out.setAlphaF(a.alphaF() + (b.alphaF() - a.alphaF()) * t);
  return out;
}

}  // namespace

TargetVisualizerDisplay::TargetVisualizerDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::geometry_msgs::PoseStamped>(
          "TargetVisualizer", std::move(channel),
          "automsgs.msgs.geometry_msgs.PoseStamped") {
  setProperties({});
}

TargetVisualizerDisplay::~TargetVisualizerDisplay() {
  goal_bridge_.reset();
}

void TargetVisualizerDisplay::onEnable() {
  ChannelDisplay::onEnable();
  goal_bridge_ = std::make_unique<QObject>();
  QObject::connect(
      &tools::GoalUpdater, &tools::GoalPoseUpdater::updateGoal,
      goal_bridge_.get(),
      [this](double x, double y, double theta, QString frame) {
        setGoalPose(x, y, theta, frame.toStdString());
      });
}

void TargetVisualizerDisplay::onDisable() {
  goal_bridge_.reset();
  ChannelDisplay::onDisable();
}

void TargetVisualizerDisplay::resetArrivalState() {
  arrived_ = false;
  arrival_t0_ = {};
}

void TargetVisualizerDisplay::setGoalPose(double x, double y, double yaw,
                                          const std::string& frame) {
  QVector3D local(static_cast<float>(x), static_cast<float>(y), 0.f);
  if (context_ != nullptr && !frame.empty() &&
      frame != context_->fixed_frame) {
    automsgs::msgs::geometry_msgs::TransformStamped tf;
    if (viz::LookupFixedFrameTransform(context_, frame, &tf)) {
      local = viz::TransformLocalPoint(tf, true, local);
    }
  }
  position_ = local;
  orientation_ = QQuaternion::fromAxisAndAngle(QVector3D(0.f, 0.f, 1.f),
                                               qRadiansToDegrees(yaw));
  have_pose_ = true;
  resetArrivalState();
  setStatusOk("Goal locked");
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

std::vector<common::DisplayPropertySpec>
TargetVisualizerDisplay::propertySpecs() const {
  return {{"color", "Color", "255;71;87", {}, common::DisplayPropertyKind::kColor},
          {"outer_color", "Outer Color", "255;160;180", {},
           common::DisplayPropertyKind::kColor},
          {"success_color", "Success Color", "80;220;120", {},
           common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "1.0"},
          {"inner_radius", "Inner Radius", "0.35"},
          {"outer_radius", "Outer Radius", "0.75"},
          {"inner_width", "Inner Width (m)", "0.08"},
          {"outer_width", "Outer Width (m)", "0.06"},
          {"rings", "Ring Segments", "64"},
          {"z_lift", "Z Lift (m)", "0.12"},
          {"show_heading", "Show Heading", "true", {"true", "false"}},
          {"heading_length", "Heading Length", "0.45"},
          {"auto_clear", "Arrive Morph + Clear", "true", {"true", "false"}},
          {"auto_clear_distance", "Arrive Distance (m)", "0.30"},
          {"arrive_morph_sec", "Arrive Morph (s)", "0.85"},
          {"success_hold_sec", "Success Hold (s)", "1.40"},
          {"robot_frame", "Robot Frame", "base_link"},
          {"show_axes", "Show Axes", "false", {"true", "false"}},
          {"axis_length", "Axis Length", "0.25"}};
}

void TargetVisualizerDisplay::clearReceivedData() {
  have_pose_ = false;
  position_ = QVector3D();
  orientation_ = QQuaternion();
  resetArrivalState();
}

bool TargetVisualizerDisplay::robotNearGoal(float distance_m) const {
  if (!have_pose_ || context_ == nullptr || context_->tf_buffer == nullptr) {
    return false;
  }
  const std::string preferred = propertyValue("robot_frame", "base_link");
  const std::string candidates[] = {preferred, "base_link", "base_footprint"};
  for (const auto& frame : candidates) {
    if (frame.empty()) {
      continue;
    }
    automsgs::msgs::geometry_msgs::TransformStamped tf;
    if (!viz::LookupFixedFrameTransform(context_, frame, &tf)) {
      continue;
    }
    const float dx =
        static_cast<float>(tf.transform().translation().x()) - position_.x();
    const float dy =
        static_cast<float>(tf.transform().translation().y()) - position_.y();
    if ((dx * dx + dy * dy) <= distance_m * distance_m) {
      return true;
    }
  }
  return false;
}

void TargetVisualizerDisplay::onUpdate() {
  // Must drain /goal_pose — skipping ChannelDisplay::onUpdate left rings blank.
  ChannelDisplay::onUpdate();

  if (have_pose_ &&
      common::ParseBoolProperty(propertyValue("auto_clear", "true"), true)) {
    const float clear_d = std::max(
        0.05f, common::ParseFloatProperty(
                   propertyValue("auto_clear_distance", "0.30"), 0.30f));
    if (!arrived_ && robotNearGoal(clear_d)) {
      arrived_ = true;
      arrival_t0_ = std::chrono::steady_clock::now();
      setStatusOk("Goal arrived — morphing");
    }

    if (arrived_) {
      const float morph_sec = std::max(
          0.15f, common::ParseFloatProperty(
                     propertyValue("arrive_morph_sec", "0.85"), 0.85f));
      const float hold_sec = std::max(
          0.f, common::ParseFloatProperty(
                   propertyValue("success_hold_sec", "1.40"), 1.40f));
      const float elapsed =
          std::chrono::duration<float>(std::chrono::steady_clock::now() -
                                       arrival_t0_)
              .count();
      if (elapsed >= morph_sec + hold_sec) {
        clearReceivedData();
        setStatusOk("Goal cleared (arrived)");
      }
    }
  }

  if (have_pose_ && context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void TargetVisualizerDisplay::processMessage(
    const automsgs::msgs::geometry_msgs::PoseStamped& message) {
  if (context_ == nullptr) {
    return;
  }
  automsgs::msgs::geometry_msgs::TransformStamped tf;
  const bool have_tf = viz::LookupFixedFrameTransform(
      context_, message.header().frame_id(), &tf);
  const QVector3D local(static_cast<float>(message.pose().position().x()),
                        static_cast<float>(message.pose().position().y()),
                        static_cast<float>(message.pose().position().z()));
  if (have_tf) {
    position_ = viz::TransformLocalPoint(tf, true, local);
  } else if (message.header().frame_id().empty() ||
             message.header().frame_id() == context_->fixed_frame) {
    position_ = local;
  } else {
    position_ = local;
    setStatusWarn("Goal TF missing; drawing in message frame");
  }
  orientation_ = viz::QuatFromProto(message.pose().orientation());
  have_pose_ = true;
  resetArrivalState();
  setStatusOk("Goal locked");
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void TargetVisualizerDisplay::clearDrawnGeometry(
    rendering::SceneOverlay& scene) const {
  drawBillboardStripOgreOrGl(context_, scene, name() + "/ring_halo", {},
                             QColor(0, 0, 0, 0), 0.01f);
  drawBillboardStripOgreOrGl(context_, scene, name() + "/ring_outer", {},
                             QColor(0, 0, 0, 0), 0.01f);
  drawBillboardStripOgreOrGl(context_, scene, name() + "/ring_mid", {},
                             QColor(0, 0, 0, 0), 0.01f);
  drawBillboardStripOgreOrGl(context_, scene, name() + "/ring_inner", {},
                             QColor(0, 0, 0, 0), 0.01f);
  drawBillboardStripOgreOrGl(context_, scene, name() + "/ring_inner_glow", {},
                             QColor(0, 0, 0, 0), 0.01f);
  drawMeshesOgreOrGl(context_, scene, name() + "/hub", {});
  drawMeshesOgreOrGl(context_, scene, name() + "/heading", {});
  drawLineSegmentsOgreOrGl(context_, scene, name() + "/rings", {});
  drawLineSegmentsOgreOrGl(context_, scene, name() + "/axes", {});
}

void TargetVisualizerDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (!have_pose_) {
    clearDrawnGeometry(scene);
    return;
  }

  QColor inner_color = common::ParseColorProperty(
      propertyValue("color", "255;71;87"), QColor(255, 71, 87));
  QColor outer_color = common::ParseColorProperty(
      propertyValue("outer_color", "255;160;180"), QColor(255, 160, 180));
  QColor success_color = common::ParseColorProperty(
      propertyValue("success_color", "80;220;120"), QColor(80, 220, 120));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);
  const float inner_r =
      common::ParseFloatProperty(propertyValue("inner_radius", "0.35"), 0.35f);
  const float outer_r =
      common::ParseFloatProperty(propertyValue("outer_radius", "0.75"), 0.75f);
  const float inner_w =
      common::ParseFloatProperty(propertyValue("inner_width", "0.08"), 0.08f);
  const float outer_w =
      common::ParseFloatProperty(propertyValue("outer_width", "0.06"), 0.06f);
  const float z_lift =
      common::ParseFloatProperty(propertyValue("z_lift", "0.12"), 0.12f);
  const int segments = std::max(
      32, common::ParseIntProperty(propertyValue("rings", "64"), 64));
  const float morph_sec = std::max(
      0.15f, common::ParseFloatProperty(
                 propertyValue("arrive_morph_sec", "0.85"), 0.85f));

  float morph = 0.f;
  if (arrived_) {
    const float elapsed =
        std::chrono::duration<float>(std::chrono::steady_clock::now() -
                                     arrival_t0_)
            .count();
    morph = Smoothstep(elapsed / morph_sec);
  }

  const float t =
      static_cast<float>(QTime::currentTime().msecsSinceStartOfDay()) * 0.0035f;
  const float pulse = 0.5f + 0.5f * static_cast<float>(std::sin(t));
  const float breath = 0.5f + 0.5f * static_cast<float>(std::sin(t * 0.65f + 1.1f));

  // Morph: amber/red → green; inner expands to outer radius; outer fades away.
  const QColor active = LerpColor(inner_color, success_color, morph);
  const QColor active_soft = LerpColor(outer_color, success_color, morph);
  const float single_r = inner_r + (outer_r - inner_r) * morph;
  const float single_w = inner_w + (outer_w * 1.15f - inner_w) * morph;
  const float outer_fade = 1.f - morph;

  // Outer / halo — fade out during arrival morph.
  if (outer_fade > 0.02f) {
    QColor halo = active_soft;
    halo.setAlphaF(alpha * (0.16f + 0.12f * pulse) * outer_fade);
    drawBillboardStripOgreOrGl(
        context_, scene, name() + "/ring_halo",
        MakeRingPolyline(position_, outer_r * (1.10f + 0.08f * pulse), z_lift,
                         segments),
        halo, std::max(0.05f, outer_w * (2.0f + 0.5f * pulse)));

    QColor outer_hi = active_soft;
    outer_hi.setAlphaF(alpha * (0.62f + 0.32f * pulse) * outer_fade);
    drawBillboardStripOgreOrGl(
        context_, scene, name() + "/ring_outer",
        MakeRingPolyline(position_, outer_r * (0.97f + 0.06f * breath),
                         z_lift + 0.01f, segments),
        outer_hi, std::max(0.04f, outer_w * (1.1f + 0.2f * breath)));
  } else {
    drawBillboardStripOgreOrGl(context_, scene, name() + "/ring_halo", {},
                               QColor(0, 0, 0, 0), 0.01f);
    drawBillboardStripOgreOrGl(context_, scene, name() + "/ring_outer", {},
                               QColor(0, 0, 0, 0), 0.01f);
  }

  drawBillboardStripOgreOrGl(context_, scene, name() + "/ring_mid", {},
                             QColor(0, 0, 0, 0), 0.01f);

  // Inner ring expands into the single success outer ring.
  QColor inner_hi = active;
  inner_hi.setAlphaF(alpha);
  drawBillboardStripOgreOrGl(
      context_, scene, name() + "/ring_inner",
      MakeRingPolyline(position_, single_r, z_lift + 0.02f, segments), inner_hi,
      std::max(0.05f, single_w));

  QColor inner_glow = active;
  inner_glow.setAlphaF(alpha * (0.28f + 0.22f * morph));
  drawBillboardStripOgreOrGl(
      context_, scene, name() + "/ring_inner_glow",
      MakeRingPolyline(position_, single_r * (1.06f + 0.04f * morph),
                       z_lift + 0.018f, segments),
      inner_glow, std::max(0.03f, single_w * 0.55f));

  // Hub: shrink slightly and turn green with the morph.
  static const ObjMesh kHub = buildSphereMesh(0.5f, 20, 14);
  QMatrix4x4 hub_tf;
  hub_tf.setToIdentity();
  hub_tf.translate(position_ + QVector3D(0.f, 0.f, z_lift + 0.03f));
  const float hub_d =
      std::max(0.05f, inner_r * 0.28f * (1.f - 0.25f * morph));
  hub_tf.scale(hub_d);
  QColor hub_color = active;
  hub_color.setAlphaF(alpha);
  drawMeshesOgreOrGl(context_, scene, name() + "/hub",
                     {{&kHub, hub_tf, hub_color, false}});

  drawLineSegmentsOgreOrGl(context_, scene, name() + "/rings", {});

  QMatrix4x4 pose_matrix;
  pose_matrix.setToIdentity();
  pose_matrix.translate(position_);
  pose_matrix.rotate(orientation_);
  const auto to_world = [&pose_matrix](const QVector3D& local) {
    return (pose_matrix * QVector4D(local, 1.f)).toVector3D();
  };

  // Heading fades out once arrived (goal complete).
  const bool show_heading =
      propertyValue("show_heading", "true") == "true" && morph < 0.85f;
  if (show_heading) {
    const float heading_len = common::ParseFloatProperty(
        propertyValue("heading_length", "0.45"), 0.45f);
    const float heading_r = single_r;
    const QVector3D start =
        to_world(QVector3D(heading_r * 0.55f, 0.f, z_lift + 0.04f));
    const QVector3D tip =
        to_world(QVector3D(heading_r + heading_len, 0.f, z_lift + 0.04f));
    QColor heading_c = active;
    heading_c.setAlphaF(alpha * (1.f - morph));
    const float shaft_d = std::max(0.035f, inner_w * 0.55f);
    const float head_d = std::max(0.07f, inner_w * 1.15f);
    drawArrowOgreOrGl(context_, scene, name() + "/heading", start, tip,
                      heading_c, 0.42f, shaft_d, head_d);
  } else {
    drawMeshesOgreOrGl(context_, scene, name() + "/heading", {});
  }

  if (propertyValue("show_axes", "false") == "true") {
    const float axis_length =
        common::ParseFloatProperty(propertyValue("axis_length", "0.25"), 0.25f);
    QColor rx(220, 60, 60);
    QColor ry(60, 220, 60);
    QColor rz(60, 120, 220);
    rx.setAlphaF(alpha);
    ry.setAlphaF(alpha);
    rz.setAlphaF(alpha);
    std::vector<LineSegment3D> axes = {
        {position_, to_world(QVector3D(axis_length, 0.f, 0.f)), rx},
        {position_, to_world(QVector3D(0.f, axis_length, 0.f)), ry},
        {position_, to_world(QVector3D(0.f, 0.f, axis_length)), rz}};
    drawLineSegmentsOgreOrGl(context_, scene, name() + "/axes", axes);
  } else {
    drawLineSegmentsOgreOrGl(context_, scene, name() + "/axes", {});
  }
}

}  // namespace display
}  // namespace autoviz

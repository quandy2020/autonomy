/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/pose/target_visualizer_display.hpp"

#include <utility>

#include <cmath>

#include <QMatrix4x4>
#include <QtMath>

#include "autoviz/plugins/displays/common/ogre_overlay_draw.hpp"
#include "autoviz/plugins/displays/common/display_tf_utils.hpp"

namespace autoviz {
namespace display {

TargetVisualizerDisplay::TargetVisualizerDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::geometry_msgs::PoseStamped>(
          "TargetVisualizer", std::move(channel),
          "automsgs.msgs.geometry_msgs.PoseStamped") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
TargetVisualizerDisplay::propertySpecs() const {
  return {{"color", "Color", "255;200;0", {}, common::DisplayPropertyKind::kColor},
          {"alpha", "Alpha", "1.0"},
          {"inner_radius", "Inner Radius", "0.2"},
          {"outer_radius", "Outer Radius", "0.5"},
          {"rings", "Ring Segments", "24"},
          {"show_axes", "Show Axes", "true", {"true", "false"}},
          {"axis_length", "Axis Length", "0.3"}};
}

void TargetVisualizerDisplay::clearReceivedData() {
  have_pose_ = false;
  position_ = QVector3D();
  orientation_ = QQuaternion();
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
  position_ = viz::TransformLocalPoint(tf, have_tf, local);
  orientation_ = viz::QuatFromProto(message.pose().orientation());
  have_pose_ = true;
  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void TargetVisualizerDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (!have_pose_) {
    return;
  }
  QColor color = common::ParseColorProperty(propertyValue("color", "255;200;0"),
                                            QColor(255, 200, 0));
  const float alpha =
      common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.f);
  color.setAlphaF(alpha);
  const float inner =
      common::ParseFloatProperty(propertyValue("inner_radius", "0.2"), 0.2f);
  const float outer =
      common::ParseFloatProperty(propertyValue("outer_radius", "0.5"), 0.5f);
  const int segments = std::max(
      8, common::ParseIntProperty(propertyValue("rings", "24"), 24));

  QMatrix4x4 pose_matrix;
  pose_matrix.setToIdentity();
  pose_matrix.translate(position_);
  pose_matrix.rotate(orientation_);
  const auto to_world = [&pose_matrix](const QVector3D& local) {
    return (pose_matrix * QVector4D(local, 1.f)).toVector3D();
  };

  std::vector<LineSegment3D> lines;
  const float radii[2] = {inner, outer};
  for (float radius : radii) {
    if (radius <= 0.f) {
      continue;
    }
    for (int s = 0; s < segments; ++s) {
      const float a0 = static_cast<float>(2.0 * M_PI * s / segments);
      const float a1 = static_cast<float>(2.0 * M_PI * (s + 1) / segments);
      const QVector3D local0(radius * qCos(a0), radius * qSin(a0), 0.f);
      const QVector3D local1(radius * qCos(a1), radius * qSin(a1), 0.f);
      lines.push_back({to_world(local0), to_world(local1), color});
    }
  }

  if (propertyValue("show_axes", "true") == "true") {
    const float axis_length =
        common::ParseFloatProperty(propertyValue("axis_length", "0.3"), 0.3f);
    const QVector3D x_end = to_world(QVector3D(axis_length, 0.f, 0.f));
    const QVector3D y_end = to_world(QVector3D(0.f, axis_length, 0.f));
    const QVector3D z_end = to_world(QVector3D(0.f, 0.f, axis_length));
    QColor rx(220, 60, 60);
    QColor ry(60, 220, 60);
    QColor rz(60, 120, 220);
    rx.setAlphaF(alpha);
    ry.setAlphaF(alpha);
    rz.setAlphaF(alpha);
    lines.push_back({position_, x_end, rx});
    lines.push_back({position_, y_end, ry});
    lines.push_back({position_, z_end, rz});
  }

  drawLineSegmentsOgreOrGl(context_, scene, name() + "/target", lines);
}

}  // namespace display
}  // namespace autoviz

/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins InteractiveMarkerControl
 * (Willow Garage / Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/displays/interactive_markers/interactive_marker_control.hpp"

#include <algorithm>
#include <cmath>
#include <map>

#include <QGenericMatrix>
#include <QMatrix3x3>

#include "autoviz/common/display_context.hpp"
#include "autoviz/plugins/displays/interactive_markers/interactive_marker.hpp"
#include "autoviz/plugins/displays/marker/marker_draw_utils.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace display {
namespace {

constexpr uint32_t kInteractionNone = 0;
constexpr uint32_t kInteractionMenu = 1;
constexpr uint32_t kInteractionButton = 2;
constexpr uint32_t kMoveAxis = 3;
constexpr uint32_t kMovePlane = 4;
constexpr uint32_t kRotateAxis = 5;
constexpr uint32_t kMoveRotate = 6;
constexpr uint32_t kMove3d = 7;
constexpr uint32_t kRotate3d = 8;
constexpr uint32_t kMoveRotate3d = 9;

QMatrix4x4 ComposePose(const QVector3D& position,
                       const QQuaternion& orientation) {
  QMatrix4x4 matrix;
  matrix.setToIdentity();
  matrix.translate(position);
  matrix.rotate(orientation);
  return matrix;
}

void DrawDefaultControlGeometry(rendering::SceneOverlay& scene,
                                const QMatrix4x4& parent_transform, float scale,
                                uint32_t interaction_mode,
                                const QColor& color) {
  const float s = std::max(scale, 0.05f);
  switch (interaction_mode) {
    case kMoveAxis: {
      const QVector3D start = parent_transform.map(QVector3D(0.f, 0.f, 0.f));
      const QVector3D end = parent_transform.map(QVector3D(s, 0.f, 0.f));
      scene.addLine(start, end, color);
      scene.addLine(end, parent_transform.map(QVector3D(0.8f * s, 0.1f * s, 0.f)),
                    color);
      scene.addLine(end,
                    parent_transform.map(QVector3D(0.8f * s, -0.1f * s, 0.f)),
                    color);
      break;
    }
    case kMovePlane:
    case kMoveRotate: {
      const QVector3D c = parent_transform.map(QVector3D(0.f, 0.f, 0.f));
      const QVector3D p0 = parent_transform.map(QVector3D(0.f, -s, -s));
      const QVector3D p1 = parent_transform.map(QVector3D(0.f, s, -s));
      const QVector3D p2 = parent_transform.map(QVector3D(0.f, s, s));
      const QVector3D p3 = parent_transform.map(QVector3D(0.f, -s, s));
      scene.addLine(p0, p1, color);
      scene.addLine(p1, p2, color);
      scene.addLine(p2, p3, color);
      scene.addLine(p3, p0, color);
      if (interaction_mode == kMoveRotate) {
        const int segments = 16;
        QVector3D prev = parent_transform.map(QVector3D(0.f, s, 0.f));
        for (int i = 1; i <= segments; ++i) {
          const float angle =
              static_cast<float>(i) * 6.2831853f / static_cast<float>(segments);
          const QVector3D next = parent_transform.map(
              QVector3D(0.f, s * std::cos(angle), s * std::sin(angle)));
          scene.addLine(prev, next, color);
          prev = next;
        }
      }
      scene.addPoint(c, color);
      break;
    }
    case kRotateAxis: {
      const int segments = 20;
      QVector3D prev = parent_transform.map(QVector3D(0.f, s, 0.f));
      for (int i = 1; i <= segments; ++i) {
        const float angle =
            static_cast<float>(i) * 6.2831853f / static_cast<float>(segments);
        const QVector3D next = parent_transform.map(
            QVector3D(0.f, s * std::cos(angle), s * std::sin(angle)));
        scene.addLine(prev, next, color);
        prev = next;
      }
      break;
    }
    case kMove3d: {
      const QVector3D center = parent_transform.map(QVector3D(0.f, 0.f, 0.f));
      scene.addPoint(center, color);
      scene.addLine(center - QVector3D(0.08f * s, 0.f, 0.f),
                    center + QVector3D(0.08f * s, 0.f, 0.f), color);
      scene.addLine(center - QVector3D(0.f, 0.08f * s, 0.f),
                    center + QVector3D(0.f, 0.08f * s, 0.f), color);
      scene.addLine(center - QVector3D(0.f, 0.f, 0.08f * s),
                    center + QVector3D(0.f, 0.f, 0.08f * s), color);
      break;
    }
    case kRotate3d: {
      const int segments = 20;
      QVector3D prev = parent_transform.map(QVector3D(0.f, s, 0.f));
      for (int i = 1; i <= segments; ++i) {
        const float angle =
            static_cast<float>(i) * 6.2831853f / static_cast<float>(segments);
        const QVector3D next = parent_transform.map(
            QVector3D(0.f, s * std::cos(angle), s * std::sin(angle)));
        scene.addLine(prev, next, color);
        prev = next;
      }
      prev = parent_transform.map(QVector3D(s, 0.f, 0.f));
      for (int i = 1; i <= segments; ++i) {
        const float angle =
            static_cast<float>(i) * 6.2831853f / static_cast<float>(segments);
        const QVector3D next = parent_transform.map(
            QVector3D(s * std::cos(angle), 0.f, s * std::sin(angle)));
        scene.addLine(prev, next, color);
        prev = next;
      }
      scene.addPoint(parent_transform.map(QVector3D(0.f, 0.f, 0.f)), color);
      break;
    }
    case kMoveRotate3d: {
      const QVector3D center = parent_transform.map(QVector3D(0.f, 0.f, 0.f));
      scene.addPoint(center, color);
      scene.addLine(center - QVector3D(0.1f * s, 0.f, 0.f),
                    center + QVector3D(0.1f * s, 0.f, 0.f),
                    QColor(255, 100, 100));
      scene.addLine(center - QVector3D(0.f, 0.1f * s, 0.f),
                    center + QVector3D(0.f, 0.1f * s, 0.f),
                    QColor(100, 255, 100));
      scene.addLine(center - QVector3D(0.f, 0.f, 0.1f * s),
                    center + QVector3D(0.f, 0.f, 0.1f * s),
                    QColor(100, 160, 255));
      break;
    }
    case kInteractionButton:
      scene.addPoint(parent_transform.map(QVector3D(0.f, 0.f, 0.f)), color);
      break;
    case kInteractionMenu:
    case kInteractionNone:
    default:
      break;
  }
}

}  // namespace

InteractiveMarkerControl::InteractiveMarkerControl(
    common::DisplayContext* context, InteractiveMarker* parent)
    : context_(context), parent_(parent) {
  control_orientation_ = QQuaternion(1.f, 0.f, 0.f, 0.f);
  parent_orientation_ = QQuaternion(1.f, 0.f, 0.f, 0.f);
  control_transform_.setToIdentity();
}

void InteractiveMarkerControl::processMessage(
    const automsgs::msgs::visualization_msgs::InteractiveMarkerControl&
        message) {
  message_ = message;
  name_ = message.name();
  description_ = QString::fromStdString(message.description());
  interaction_mode_ = static_cast<int>(message.interaction_mode());
  orientation_mode_ = static_cast<int>(message.orientation_mode());
  independent_marker_orientation_ = message.independent_marker_orientation();
  always_visible_ = message.always_visible();

  const auto& q = message.orientation();
  if (q.w() == 0.0 && q.x() == 0.0 && q.y() == 0.0 && q.z() == 0.0) {
    control_orientation_ = QQuaternion(1.f, 0.f, 0.f, 0.f);
  } else {
    control_orientation_ =
        QQuaternion(static_cast<float>(q.w()), static_cast<float>(q.x()),
                    static_cast<float>(q.y()), static_cast<float>(q.z()));
  }

  markers_.clear();
  markers_.reserve(static_cast<std::size_t>(message.markers_size()));
  for (const auto& marker : message.markers()) {
    markers_.push_back(marker);
  }

  if (parent_ != nullptr) {
    parent_position_ = parent_->getPosition();
    parent_orientation_ = parent_->getOrientation();
  }
  rebuildControlTransform();
}

void InteractiveMarkerControl::interactiveMarkerPoseChanged(
    const QVector3D& int_marker_position,
    const QQuaternion& int_marker_orientation) {
  parent_position_ = int_marker_position;
  parent_orientation_ = int_marker_orientation;
  rebuildControlTransform();
}

bool InteractiveMarkerControl::isInteractive() const {
  return interaction_mode_ !=
         static_cast<int>(
             automsgs::msgs::visualization_msgs::InteractiveMarkerControl::
                 NONE);
}

void InteractiveMarkerControl::update() { rebuildControlTransform(); }

void InteractiveMarkerControl::setVisible(bool visible) { visible_ = visible; }

void InteractiveMarkerControl::setHighlight(ControlHighlight highlight) {
  highlight_ = static_cast<float>(highlight) * 0.1f;
}

void InteractiveMarkerControl::rebuildControlTransform() {
  // INHERIT: control frame follows parent; FIXED / VIEW_FACING use control ori.
  QQuaternion orientation = parent_orientation_;
  using Mode = automsgs::msgs::visualization_msgs::InteractiveMarkerControl;
  if (orientation_mode_ == Mode::FIXED ||
      orientation_mode_ == Mode::VIEW_FACING) {
    orientation = parent_orientation_ * control_orientation_;
  } else {
    orientation = parent_orientation_ * control_orientation_;
  }
  control_transform_ = ComposePose(parent_position_, orientation);

  // Lift into fixed frame when parent pose is expressed in reference frame.
  if (parent_ != nullptr && context_ != nullptr &&
      !parent_->getReferenceFrame().empty()) {
    automsgs::msgs::visualization_msgs::Marker pose_marker;
    pose_marker.mutable_header()->set_frame_id(parent_->getReferenceFrame());
    pose_marker.mutable_pose()->mutable_position()->set_x(parent_position_.x());
    pose_marker.mutable_pose()->mutable_position()->set_y(parent_position_.y());
    pose_marker.mutable_pose()->mutable_position()->set_z(parent_position_.z());
    pose_marker.mutable_pose()->mutable_orientation()->set_w(
        parent_orientation_.scalar());
    pose_marker.mutable_pose()->mutable_orientation()->set_x(
        parent_orientation_.x());
    pose_marker.mutable_pose()->mutable_orientation()->set_y(
        parent_orientation_.y());
    pose_marker.mutable_pose()->mutable_orientation()->set_z(
        parent_orientation_.z());
    const QMatrix4x4 parent_tf =
        markerTransformInFixedFrame(pose_marker, context_);
    QMatrix4x4 local;
    local.setToIdentity();
    local.rotate(control_orientation_);
    control_transform_ = parent_tf * local;
  }
}

void InteractiveMarkerControl::draw(
    rendering::SceneOverlay& scene,
    const common::DisplayPropertyMap& properties,
    const std::string& draw_prefix) const {
  if (!visible_ && !always_visible_) {
    return;
  }
  const float scale = parent_ != nullptr ? parent_->getSize() : 0.2f;
  QColor color(255, 220, 80);
  if (highlight_ > 0.f) {
    color = color.lighter(100 + static_cast<int>(highlight_ * 80.f));
  }

  if (!markers_.empty()) {
    std::map<MarkerKey, StoredMarker> control_markers;
    for (const auto& embedded : markers_) {
      // Embedded marker pose is relative to the control frame (RViz).
      const QVector3D local_pos(
          static_cast<float>(embedded.pose().position().x()),
          static_cast<float>(embedded.pose().position().y()),
          static_cast<float>(embedded.pose().position().z()));
      QQuaternion local_ori(
          static_cast<float>(embedded.pose().orientation().w()),
          static_cast<float>(embedded.pose().orientation().x()),
          static_cast<float>(embedded.pose().orientation().y()),
          static_cast<float>(embedded.pose().orientation().z()));
      if (local_ori.isNull()) {
        local_ori = QQuaternion(1.f, 0.f, 0.f, 0.f);
      }
      const QVector3D pos = control_transform_.map(local_pos);
      QQuaternion control_ori;
      {
        // Extract rotation from control_transform_ columns.
        const QVector3D x = control_transform_.column(0).toVector3D().normalized();
        const QVector3D y = control_transform_.column(1).toVector3D().normalized();
        const QVector3D z = control_transform_.column(2).toVector3D().normalized();
        QMatrix3x3 rot;
        rot(0, 0) = x.x();
        rot(1, 0) = x.y();
        rot(2, 0) = x.z();
        rot(0, 1) = y.x();
        rot(1, 1) = y.y();
        rot(2, 1) = y.z();
        rot(0, 2) = z.x();
        rot(1, 2) = z.y();
        rot(2, 2) = z.z();
        control_ori = QQuaternion::fromRotationMatrix(rot);
      }
      const QQuaternion ori = control_ori * local_ori;

      automsgs::msgs::visualization_msgs::Marker transformed = embedded;
      transformed.mutable_header()->set_frame_id(
          context_ != nullptr ? context_->fixed_frame : std::string());
      transformed.mutable_pose()->mutable_position()->set_x(pos.x());
      transformed.mutable_pose()->mutable_position()->set_y(pos.y());
      transformed.mutable_pose()->mutable_position()->set_z(pos.z());
      transformed.mutable_pose()->mutable_orientation()->set_w(ori.scalar());
      transformed.mutable_pose()->mutable_orientation()->set_x(ori.x());
      transformed.mutable_pose()->mutable_orientation()->set_y(ori.y());
      transformed.mutable_pose()->mutable_orientation()->set_z(ori.z());
      upsertMarker(transformed, &control_markers);
    }
    drawStoredMarkers(scene, context_, properties, control_markers,
                      draw_prefix);
  } else {
    DrawDefaultControlGeometry(scene, control_transform_, scale,
                               static_cast<uint32_t>(interaction_mode_), color);
  }

  if (show_visual_aids_ && isInteractive()) {
    scene.addPoint(control_transform_.map(QVector3D(0.f, 0.f, 0.f)),
                   QColor(255, 255, 255));
  }
}

void InteractiveMarkerControl::collectPickSamples(
    float marker_scale, std::vector<QVector3D>* samples) const {
  if (samples == nullptr || !isInteractive() ||
      interaction_mode_ == static_cast<int>(kInteractionMenu)) {
    return;
  }
  const float s = std::max(marker_scale, 0.05f);
  switch (static_cast<uint32_t>(interaction_mode_)) {
    case kMoveAxis:
      samples->push_back(control_transform_.map(QVector3D(0.f, 0.f, 0.f)));
      samples->push_back(control_transform_.map(QVector3D(s, 0.f, 0.f)));
      break;
    case kMovePlane:
    case kMoveRotate:
      samples->push_back(control_transform_.map(QVector3D(0.f, 0.f, 0.f)));
      samples->push_back(control_transform_.map(QVector3D(0.f, -s, -s)));
      samples->push_back(control_transform_.map(QVector3D(0.f, s, s)));
      break;
    case kRotateAxis:
    case kRotate3d: {
      const int segments = 8;
      for (int i = 0; i <= segments; ++i) {
        const float angle =
            static_cast<float>(i) * 6.2831853f / static_cast<float>(segments);
        samples->push_back(control_transform_.map(
            QVector3D(0.f, s * std::cos(angle), s * std::sin(angle))));
      }
      break;
    }
    case kMove3d:
    case kMoveRotate3d:
    case kInteractionButton:
      samples->push_back(control_transform_.map(QVector3D(0.f, 0.f, 0.f)));
      break;
    default:
      break;
  }
}

}  // namespace display
}  // namespace autoviz

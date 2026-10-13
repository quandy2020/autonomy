/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/interactive_markers/interactive_marker_registry.hpp"

#include <algorithm>
#include <cmath>

#include <QMatrix3x3>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector4D>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autoviz/common/display_context.hpp"
#include "autoviz/plugins/displays/marker/marker_draw_utils.hpp"
#include "autoviz/plugins/displays/common/transform_utils.hpp"

namespace autoviz {
namespace display {
namespace {

constexpr uint32_t kUpdateType = 1;
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

QMatrix4x4 MarkerPoseMatrix(
    const automsgs::msgs::visualization_msgs::InteractiveMarker& marker,
    common::DisplayContext* context) {
  automsgs::msgs::visualization_msgs::Marker pose_marker;
  pose_marker.mutable_header()->set_frame_id(marker.header().frame_id());
  pose_marker.mutable_pose()->CopyFrom(marker.pose());
  return markerPoseTransformInFixedFrame(pose_marker, context);
}

QQuaternion ControlOrientationQuat(
    const automsgs::msgs::visualization_msgs::InteractiveMarkerControl&
        control) {
  const auto& q = control.orientation();
  if (q.w() == 0.0 && q.x() == 0.0 && q.y() == 0.0 && q.z() == 0.0) {
    return QQuaternion(1.f, 0.f, 0.f, 0.f);
  }
  return QQuaternion(static_cast<float>(q.w()), static_cast<float>(q.x()),
                     static_cast<float>(q.y()), static_cast<float>(q.z()))
      .normalized();
}

QQuaternion ViewFacingOrientation(const QQuaternion& control_orientation,
                                  const QVector3D& cam_forward,
                                  const QVector3D& cam_up) {
  const QVector3D forward = cam_forward.normalized();
  QVector3D up = cam_up.normalized();
  if (forward.lengthSquared() < 1e-12f) {
    return control_orientation;
  }
  if (up.lengthSquared() < 1e-12f) {
    up = QVector3D(0.f, 0.f, 1.f);
  }
  const QVector3D x_axis =
      control_orientation.rotatedVector(QVector3D(1.f, 0.f, 0.f));
  const QVector3D z_axis =
      control_orientation.rotatedVector(QVector3D(0.f, 0.f, 1.f));
  const QQuaternion x_view_facing = QQuaternion::rotationTo(x_axis, forward);
  const QVector3D z_axis_2 = x_view_facing.rotatedVector(z_axis);
  const QQuaternion align_yz = QQuaternion::rotationTo(z_axis_2, up);
  return align_yz * x_view_facing;
}

QMatrix4x4 ControlTransform(
    const automsgs::msgs::visualization_msgs::InteractiveMarker& marker,
    const automsgs::msgs::visualization_msgs::InteractiveMarkerControl&
        control,
    common::DisplayContext* context) {
  // Match InteractiveMarkerControl::rebuildControlTransform (RViz modes).
  using Mode = automsgs::msgs::visualization_msgs::InteractiveMarkerControl;
  const QMatrix4x4 parent_tf = MarkerPoseMatrix(marker, context);
  const QVector3D parent_pos = parent_tf.map(QVector3D(0.f, 0.f, 0.f));
  const QVector3D px = parent_tf.column(0).toVector3D().normalized();
  const QVector3D py = parent_tf.column(1).toVector3D().normalized();
  const QVector3D pz = parent_tf.column(2).toVector3D().normalized();
  QMatrix3x3 prot;
  prot(0, 0) = px.x();
  prot(1, 0) = px.y();
  prot(2, 0) = px.z();
  prot(0, 1) = py.x();
  prot(1, 1) = py.y();
  prot(2, 1) = py.z();
  prot(0, 2) = pz.x();
  prot(1, 2) = pz.y();
  prot(2, 2) = pz.z();
  const QQuaternion parent_ori = QQuaternion::fromRotationMatrix(prot);
  const QQuaternion control_ori = ControlOrientationQuat(control);

  QQuaternion control_frame_ori = parent_ori;
  if (control.orientation_mode() == Mode::FIXED) {
    control_frame_ori = QQuaternion(1.f, 0.f, 0.f, 0.f);
  } else if (control.orientation_mode() == Mode::VIEW_FACING) {
    QVector3D cam_forward(0.f, 0.f, -1.f);
    QVector3D cam_up(0.f, 0.f, 1.f);
    if (context != nullptr) {
      if (context->view_controller != nullptr) {
        const QMatrix4x4 inv = context->view_controller->viewMatrix().inverted();
        cam_forward =
            (inv * QVector4D(0.f, 0.f, -1.f, 0.f)).toVector3D().normalized();
        cam_up = (inv * QVector4D(0.f, 1.f, 0.f, 0.f)).toVector3D().normalized();
      } else if (context->has_view_matrices) {
        const QMatrix4x4 inv = context->view_matrix.inverted();
        cam_forward =
            (inv * QVector4D(0.f, 0.f, -1.f, 0.f)).toVector3D().normalized();
        cam_up = (inv * QVector4D(0.f, 1.f, 0.f, 0.f)).toVector3D().normalized();
      }
    }
    control_frame_ori =
        ViewFacingOrientation(control_ori, cam_forward, cam_up);
  }

  QMatrix4x4 control_transform;
  control_transform.setToIdentity();
  control_transform.translate(parent_pos);
  control_transform.rotate(control_frame_ori * control_ori);
  return control_transform;
}

QVector3D PosePosition(
    const automsgs::msgs::geometry_msgs::Pose& pose) {
  return QVector3D(static_cast<float>(pose.position().x()),
                     static_cast<float>(pose.position().y()),
                     static_cast<float>(pose.position().z()));
}

void SetPosePosition(automsgs::msgs::geometry_msgs::Pose* pose,
                     const QVector3D& position) {
  pose->mutable_position()->set_x(position.x());
  pose->mutable_position()->set_y(position.y());
  pose->mutable_position()->set_z(position.z());
}

QQuaternion PoseOrientation(
    const automsgs::msgs::geometry_msgs::Pose& pose) {
  return QQuaternion(static_cast<float>(pose.orientation().w()),
                     static_cast<float>(pose.orientation().x()),
                     static_cast<float>(pose.orientation().y()),
                     static_cast<float>(pose.orientation().z()));
}

void SetPoseOrientation(automsgs::msgs::geometry_msgs::Pose* pose,
                        const QQuaternion& orientation) {
  pose->mutable_orientation()->set_w(orientation.scalar());
  pose->mutable_orientation()->set_x(orientation.x());
  pose->mutable_orientation()->set_y(orientation.y());
  pose->mutable_orientation()->set_z(orientation.z());
}

QVector3D AxisFromTransform(const QMatrix4x4& transform, int column) {
  return transform.column(column).toVector3D().normalized();
}

/**
 * Pick samples matching interactive_markers MakeArrow / MakeDisc geometry.
 * Disc: inner 0.5×scale, outer 0.65×scale in YZ. Arrow: ±0.5..0.9×scale on X.
 */
void CollectControlPickSamples(
    const QMatrix4x4& control_transform, float scale, uint32_t interaction_mode,
    std::vector<QVector3D>* samples) {
  const float s = std::max(scale, 0.05f);
  switch (interaction_mode) {
    case kMoveAxis: {
      // MakeArrow shafts at ±[0.5, 0.9] * scale along control X.
      for (float dir : {1.f, -1.f}) {
        samples->push_back(
            control_transform.map(QVector3D(dir * 0.5f * s, 0.f, 0.f)));
        samples->push_back(
            control_transform.map(QVector3D(dir * 0.7f * s, 0.f, 0.f)));
        samples->push_back(
            control_transform.map(QVector3D(dir * 0.9f * s, 0.f, 0.f)));
      }
      samples->push_back(control_transform.map(QVector3D(0.f, 0.f, 0.f)));
      break;
    }
    case kMovePlane:
    case kMoveRotate:
    case kRotateAxis: {
      // MakeDisc ring in YZ plane (inner 0.5, outer 0.65 of unit × scale).
      constexpr int kSegments = 16;
      for (float radius : {0.5f * s, 0.65f * s}) {
        for (int i = 0; i <= kSegments; ++i) {
          const float angle = static_cast<float>(i) * 6.2831853f /
                              static_cast<float>(kSegments);
          samples->push_back(control_transform.map(
              QVector3D(0.f, radius * std::cos(angle),
                        radius * std::sin(angle))));
        }
      }
      samples->push_back(control_transform.map(QVector3D(0.f, 0.f, 0.f)));
      break;
    }
    case kRotate3d: {
      constexpr int kSegments = 12;
      for (int i = 0; i <= kSegments; ++i) {
        const float angle = static_cast<float>(i) * 6.2831853f /
                            static_cast<float>(kSegments);
        const float c = 0.5f * s * std::cos(angle);
        const float sn = 0.5f * s * std::sin(angle);
        samples->push_back(control_transform.map(QVector3D(0.f, c, sn)));
        samples->push_back(control_transform.map(QVector3D(c, 0.f, sn)));
        samples->push_back(control_transform.map(QVector3D(c, sn, 0.f)));
      }
      break;
    }
    case kMove3d:
    case kMoveRotate3d:
    case kInteractionButton:
      samples->push_back(control_transform.map(QVector3D(0.f, 0.f, 0.f)));
      break;
    default:
      break;
  }
}

/** Samples embedded control markers in the interactive-marker (markers_node) frame. */
void CollectEmbeddedMarkerPickSamples(
    const QMatrix4x4& markers_transform,
    const automsgs::msgs::visualization_msgs::Marker& embedded,
    std::vector<QVector3D>* samples) {
  if (samples == nullptr) {
    return;
  }
  QMatrix4x4 local;
  local.setToIdentity();
  local.translate(static_cast<float>(embedded.pose().position().x()),
                  static_cast<float>(embedded.pose().position().y()),
                  static_cast<float>(embedded.pose().position().z()));
  local.rotate(QQuaternion(
      static_cast<float>(embedded.pose().orientation().w()),
      static_cast<float>(embedded.pose().orientation().x()),
      static_cast<float>(embedded.pose().orientation().y()),
      static_cast<float>(embedded.pose().orientation().z())));
  const QMatrix4x4 world = markers_transform * local;
  constexpr int32_t kCube = 1;
  constexpr int32_t kSphere = 2;
  constexpr int32_t kArrow = 0;
  constexpr int32_t kTriangleList = 11;
  if (embedded.type() == kCube || embedded.type() == kSphere) {
    const float hx = 0.5f * static_cast<float>(embedded.scale().x());
    const float hy = 0.5f * static_cast<float>(embedded.scale().y());
    const float hz = 0.5f * static_cast<float>(embedded.scale().z());
    for (float sx : {-hx, 0.f, hx}) {
      for (float sy : {-hy, 0.f, hy}) {
        for (float sz : {-hz, 0.f, hz}) {
          samples->push_back(world.map(QVector3D(sx, sy, sz)));
        }
      }
    }
  } else if (embedded.type() == kArrow && embedded.points_size() >= 2) {
    for (const auto& p : embedded.points()) {
      samples->push_back(world.map(QVector3D(static_cast<float>(p.x()),
                                             static_cast<float>(p.y()),
                                             static_cast<float>(p.z()))));
    }
  } else if (embedded.type() == kTriangleList && embedded.points_size() > 0) {
    // Subsample disc / mesh vertices (already in unit space; scale via TF).
    QMatrix4x4 scaled = world;
    scaled.scale(std::max(0.01f, static_cast<float>(embedded.scale().x())),
                 std::max(0.01f, static_cast<float>(embedded.scale().y())),
                 std::max(0.01f, static_cast<float>(embedded.scale().z())));
    const int stride = std::max(1, embedded.points_size() / 24);
    for (int i = 0; i < embedded.points_size(); i += stride) {
      const auto& p = embedded.points(i);
      samples->push_back(scaled.map(QVector3D(static_cast<float>(p.x()),
                                              static_cast<float>(p.y()),
                                              static_cast<float>(p.z()))));
    }
  } else {
    samples->push_back(world.map(QVector3D(0.f, 0.f, 0.f)));
  }
}

QMatrix4x4 MarkersTransform(
    const automsgs::msgs::visualization_msgs::InteractiveMarker& marker,
    const automsgs::msgs::visualization_msgs::InteractiveMarkerControl&
        control,
    common::DisplayContext* context) {
  // RViz markers_node_: INHERIT=parent, FIXED=identity, VIEW_FACING=view rot
  // (autoComplete puts control.orientation on each marker pose).
  using Mode = automsgs::msgs::visualization_msgs::InteractiveMarkerControl;
  const QMatrix4x4 parent_tf = MarkerPoseMatrix(marker, context);
  const QVector3D parent_pos = parent_tf.map(QVector3D(0.f, 0.f, 0.f));
  const QVector3D px = parent_tf.column(0).toVector3D().normalized();
  const QVector3D py = parent_tf.column(1).toVector3D().normalized();
  const QVector3D pz = parent_tf.column(2).toVector3D().normalized();
  QMatrix3x3 prot;
  prot(0, 0) = px.x();
  prot(1, 0) = px.y();
  prot(2, 0) = px.z();
  prot(0, 1) = py.x();
  prot(1, 1) = py.y();
  prot(2, 1) = py.z();
  prot(0, 2) = pz.x();
  prot(1, 2) = pz.y();
  prot(2, 2) = pz.z();
  const QQuaternion parent_ori = QQuaternion::fromRotationMatrix(prot);
  QQuaternion markers_ori = parent_ori;
  if (control.orientation_mode() == Mode::FIXED) {
    markers_ori = QQuaternion(1.f, 0.f, 0.f, 0.f);
  } else if (control.orientation_mode() == Mode::VIEW_FACING) {
    if (control.independent_marker_orientation()) {
      markers_ori = parent_ori;
    } else {
      QVector3D cam_forward(0.f, 0.f, -1.f);
      QVector3D cam_up(0.f, 0.f, 1.f);
      if (context != nullptr) {
        if (context->view_controller != nullptr) {
          const QMatrix4x4 inv =
              context->view_controller->viewMatrix().inverted();
          cam_forward =
              (inv * QVector4D(0.f, 0.f, -1.f, 0.f)).toVector3D().normalized();
          cam_up =
              (inv * QVector4D(0.f, 1.f, 0.f, 0.f)).toVector3D().normalized();
        } else if (context->has_view_matrices) {
          const QMatrix4x4 inv = context->view_matrix.inverted();
          cam_forward =
              (inv * QVector4D(0.f, 0.f, -1.f, 0.f)).toVector3D().normalized();
          cam_up =
              (inv * QVector4D(0.f, 1.f, 0.f, 0.f)).toVector3D().normalized();
        }
      }
      markers_ori = ViewFacingOrientation(ControlOrientationQuat(control),
                                          cam_forward, cam_up);
    }
  }
  QMatrix4x4 m;
  m.setToIdentity();
  m.translate(parent_pos);
  m.rotate(markers_ori);
  return m;
}

void DrawDefaultControl(rendering::SceneOverlay& scene,
                        const QMatrix4x4& parent_transform, float scale,
                        uint32_t interaction_mode, const QColor& color) {
  const float s = std::max(scale, 0.05f);
  switch (interaction_mode) {
    case kMoveAxis: {
      const QVector3D start = parent_transform.map(QVector3D(0.f, 0.f, 0.f));
      const QVector3D end = parent_transform.map(QVector3D(s, 0.f, 0.f));
      scene.addLine(start, end, color);
      scene.addLine(end, parent_transform.map(QVector3D(0.8f * s, 0.1f * s, 0.f)),
                    color);
      scene.addLine(end, parent_transform.map(QVector3D(0.8f * s, -0.1f * s, 0.f)),
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
          const float angle = static_cast<float>(i) * 6.2831853f /
                              static_cast<float>(segments);
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
        const float angle = static_cast<float>(i) * 6.2831853f /
                            static_cast<float>(segments);
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
        const float angle = static_cast<float>(i) * 6.2831853f /
                            static_cast<float>(segments);
        const QVector3D next = parent_transform.map(
            QVector3D(0.f, s * std::cos(angle), s * std::sin(angle)));
        scene.addLine(prev, next, color);
        prev = next;
      }
      prev = parent_transform.map(QVector3D(s, 0.f, 0.f));
      for (int i = 1; i <= segments; ++i) {
        const float angle = static_cast<float>(i) * 6.2831853f /
                            static_cast<float>(segments);
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

bool ProjectToScreen(const QMatrix4x4& mvp, int viewport_width,
                     int viewport_height, const QVector3D& world, float* out_x,
                     float* out_y) {
  const QVector4D clip = mvp * QVector4D(world, 1.f);
  if (clip.w() <= 1e-4f) {
    return false;
  }
  const float ndc_x = clip.x() / clip.w();
  const float ndc_y = clip.y() / clip.w();
  *out_x = (ndc_x + 1.f) * 0.5f * static_cast<float>(viewport_width);
  *out_y = (1.f - ndc_y) * 0.5f * static_cast<float>(viewport_height);
  return true;
}

}  // namespace

std::string defaultFeedbackChannel(const std::string& update_channel) {
  const std::string suffix = "/update";
  if (update_channel.size() > suffix.size() &&
      update_channel.compare(update_channel.size() - suffix.size(), suffix.size(),
                             suffix) == 0) {
    return update_channel.substr(0, update_channel.size() - suffix.size()) +
           "/feedback";
  }
  return update_channel + "/feedback";
}

void applyInteractiveMarkerUpdate(
    const automsgs::msgs::visualization_msgs::InteractiveMarkerUpdate&
        update,
    const std::string& server_id, const std::string& feedback_channel,
    const std::string& source_id,
    std::map<std::string, InteractiveMarkerState>* markers) {
  if (markers == nullptr || update.type() != kUpdateType) {
    return;
  }
  for (const auto& marker : update.markers()) {
    InteractiveMarkerState state;
    state.server_id = server_id.empty() ? update.server_id() : server_id;
    state.feedback_channel = feedback_channel;
    state.source_id = source_id;
    state.marker = marker;
    (*markers)[marker.name()] = std::move(state);
  }
  for (const auto& pose_update : update.poses()) {
    auto it = markers->find(pose_update.name());
    if (it != markers->end()) {
      it->second.marker.mutable_pose()->CopyFrom(pose_update.pose());
      if (!pose_update.header().frame_id().empty()) {
        it->second.marker.mutable_header()->set_frame_id(
            pose_update.header().frame_id());
      }
    }
  }
  for (const auto& erased : update.erases()) {
    markers->erase(erased);
  }
}

QVector3D markerPositionInFixedFrame(
    const automsgs::msgs::visualization_msgs::InteractiveMarker&
        marker,
    common::DisplayContext* context) {
  automsgs::msgs::visualization_msgs::Marker pose_marker;
  pose_marker.mutable_header()->set_frame_id(marker.header().frame_id());
  pose_marker.mutable_pose()->CopyFrom(marker.pose());
  const QMatrix4x4 transform = markerTransformInFixedFrame(pose_marker, context);
  return transform.map(QVector3D(0.f, 0.f, 0.f));
}

void drawInteractiveMarker(rendering::SceneOverlay& scene,
                           common::DisplayContext* context,
                           const common::DisplayPropertyMap& properties,
                           const InteractiveMarkerState& state) {
  const auto& marker = state.marker;
  const float scale =
      std::max(static_cast<float>(marker.scale()), 0.05f);
  const QColor handle_color(255, 220, 80);

  automsgs::msgs::visualization_msgs::Marker body;
  body.mutable_header()->set_frame_id(marker.header().frame_id());
  body.mutable_pose()->CopyFrom(marker.pose());
  body.set_type(automsgs::msgs::visualization_msgs::Marker::CUBE);
  body.set_action(automsgs::msgs::visualization_msgs::Marker::ADD);
  body.mutable_scale()->set_x(scale * 0.25);
  body.mutable_scale()->set_y(scale * 0.25);
  body.mutable_scale()->set_z(scale * 0.25);
  body.mutable_color()->set_r(0.4f);
  body.mutable_color()->set_g(0.75f);
  body.mutable_color()->set_b(1.f);
  body.mutable_color()->set_a(0.85f);
  std::map<MarkerKey, StoredMarker> body_map;
  upsertMarker(body, &body_map);
  drawStoredMarkers(scene, context, properties, body_map,
                    state.marker.name() + "/body");

  int control_index = 0;
  for (const auto& control : marker.controls()) {
    // Interaction / default gizmos use the control frame (includes control.ori).
    const QMatrix4x4 control_transform =
        ControlTransform(marker, control, context);
    if (control.markers_size() > 0) {
      // RViz markers_node_: embedded poses are relative to the interactive
      // marker (not control_frame). autoComplete already baked control.ori
      // into each marker pose — use MarkersTransform to avoid double-apply.
      const QMatrix4x4 markers_transform =
          MarkersTransform(marker, control, context);
      const QVector3D mx =
          markers_transform.column(0).toVector3D().normalized();
      const QVector3D my =
          markers_transform.column(1).toVector3D().normalized();
      const QVector3D mz =
          markers_transform.column(2).toVector3D().normalized();
      QMatrix3x3 mrot;
      mrot(0, 0) = mx.x();
      mrot(1, 0) = mx.y();
      mrot(2, 0) = mx.z();
      mrot(0, 1) = my.x();
      mrot(1, 1) = my.y();
      mrot(2, 1) = my.z();
      mrot(0, 2) = mz.x();
      mrot(1, 2) = mz.y();
      mrot(2, 2) = mz.z();
      const QQuaternion markers_ori = QQuaternion::fromRotationMatrix(mrot);

      std::map<MarkerKey, StoredMarker> control_markers;
      for (const auto& embedded : control.markers()) {
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
        const QVector3D pos = markers_transform.map(local_pos);
        const QQuaternion ori = markers_ori * local_ori;

        automsgs::msgs::visualization_msgs::Marker transformed = embedded;
        if (context != nullptr) {
          transformed.mutable_header()->set_frame_id(context->fixed_frame);
        }
        transformed.mutable_pose()->mutable_position()->set_x(pos.x());
        transformed.mutable_pose()->mutable_position()->set_y(pos.y());
        transformed.mutable_pose()->mutable_position()->set_z(pos.z());
        transformed.mutable_pose()->mutable_orientation()->set_w(ori.scalar());
        transformed.mutable_pose()->mutable_orientation()->set_x(ori.x());
        transformed.mutable_pose()->mutable_orientation()->set_y(ori.y());
        transformed.mutable_pose()->mutable_orientation()->set_z(ori.z());
        upsertMarker(transformed, &control_markers);
      }
      drawStoredMarkers(scene, context, properties, control_markers,
                        state.marker.name() + "/control/" +
                            std::to_string(control_index++));
    } else {
      DrawDefaultControl(scene, control_transform, scale, control.interaction_mode(),
                         handle_color);
      ++control_index;
    }
  }
}

void InteractiveMarkerRegistry::setSourceMarkers(
    const std::string& source_id,
    const std::map<std::string, InteractiveMarkerState>& markers) {
  for (auto it = source_by_marker_.begin(); it != source_by_marker_.end();) {
    if (it->second == source_id) {
      markers_.erase(it->first);
      it = source_by_marker_.erase(it);
    } else {
      ++it;
    }
  }
  for (const auto& [name, state] : markers) {
    markers_[name] = state;
    source_by_marker_[name] = source_id;
  }
}

void InteractiveMarkerRegistry::removeSource(const std::string& source_id) {
  for (auto it = source_by_marker_.begin(); it != source_by_marker_.end();) {
    if (it->second == source_id) {
      markers_.erase(it->first);
      it = source_by_marker_.erase(it);
    } else {
      ++it;
    }
  }
}

bool InteractiveMarkerRegistry::updatePose(
    const std::string& marker_name,
    const automsgs::msgs::geometry_msgs::Pose& pose) {
  auto it = markers_.find(marker_name);
  if (it == markers_.end()) {
    return false;
  }
  it->second.marker.mutable_pose()->CopyFrom(pose);
  return true;
}

InteractiveMarkerPick InteractiveMarkerRegistry::pickMarker(
    rendering::ViewController* view_controller, int viewport_width,
    int viewport_height, int pixel_x, int pixel_y,
    common::DisplayContext* context, float max_pixel_distance) const {
  InteractiveMarkerPick best;
  if (view_controller == nullptr || context == nullptr || markers_.empty()) {
    return best;
  }
  const float aspect =
      static_cast<float>(viewport_width) /
      static_cast<float>(std::max(1, viewport_height));
  const QMatrix4x4 mvp =
      view_controller->projectionMatrix(aspect) * view_controller->viewMatrix();

  auto consider = [&](const InteractiveMarkerPick& candidate) {
    if (!candidate.hit) {
      return;
    }
    if (!best.hit || candidate.pixel_distance < best.pixel_distance) {
      best = candidate;
    }
  };

  for (const auto& [name, state] : markers_) {
    const auto& marker = state.marker;
    const float scale = std::max(static_cast<float>(marker.scale()), 0.05f);
    const QVector3D marker_center = markerPositionInFixedFrame(marker, context);
    InteractiveMarkerPick marker_best;

    for (int i = 0; i < marker.controls_size(); ++i) {
      const auto& control = marker.controls(i);
      if (control.interaction_mode() == kInteractionNone ||
          control.interaction_mode() == kInteractionMenu) {
        continue;
      }
      const QMatrix4x4 control_transform =
          ControlTransform(marker, control, context);
      std::vector<QVector3D> samples;
      CollectControlPickSamples(control_transform, scale,
                                control.interaction_mode(), &samples);
      // Prefer real control marker geometry (autoComplete ARROW / disc / cube).
      if (control.markers_size() > 0) {
        const QMatrix4x4 markers_tf =
            MarkersTransform(marker, control, context);
        for (const auto& embedded : control.markers()) {
          CollectEmbeddedMarkerPickSamples(markers_tf, embedded, &samples);
        }
      }
      if (samples.empty()) {
        continue;
      }
      float best_control_distance = max_pixel_distance + 1.f;
      QVector3D best_sample = samples.front();
      for (const auto& sample : samples) {
        float sx = 0.f;
        float sy = 0.f;
        if (!ProjectToScreen(mvp, viewport_width, viewport_height, sample, &sx,
                             &sy)) {
          continue;
        }
        const float dx = sx - static_cast<float>(pixel_x);
        const float dy = sy - static_cast<float>(pixel_y);
        const float distance = std::sqrt(dx * dx + dy * dy);
        if (distance < best_control_distance) {
          best_control_distance = distance;
          best_sample = sample;
        }
      }
      if (best_control_distance > max_pixel_distance) {
        continue;
      }
      InteractiveMarkerPick candidate;
      candidate.hit = true;
      candidate.marker_name = name;
      candidate.control_name = control.name();
      candidate.feedback_channel = state.feedback_channel;
      candidate.frame_id = (marker.header().frame_id().empty()
                               ? context->fixed_frame
                               : marker.header().frame_id());
      candidate.position = best_sample;
      candidate.control_transform = control_transform;
      candidate.interaction_mode = control.interaction_mode();
      candidate.control_index = i;
      candidate.pixel_distance = best_control_distance;
      if (!marker_best.hit ||
          candidate.pixel_distance < marker_best.pixel_distance) {
        marker_best = candidate;
      }
    }

    if (!marker_best.hit) {
      float sx = 0.f;
      float sy = 0.f;
      if (ProjectToScreen(mvp, viewport_width, viewport_height, marker_center,
                          &sx, &sy)) {
        const float dx = sx - static_cast<float>(pixel_x);
        const float dy = sy - static_cast<float>(pixel_y);
        const float distance = std::sqrt(dx * dx + dy * dy);
        if (distance <= max_pixel_distance) {
          marker_best.hit = true;
          marker_best.marker_name = name;
          marker_best.feedback_channel = state.feedback_channel;
          marker_best.frame_id = (marker.header().frame_id().empty()
                                     ? context->fixed_frame
                                     : marker.header().frame_id());
          marker_best.position = marker_center;
          marker_best.control_transform = MarkerPoseMatrix(marker, context);
          marker_best.interaction_mode =
              marker.menu_entries().empty() ? kMove3d : kInteractionMenu;
          marker_best.pixel_distance = distance;
        }
      }
    }

    consider(marker_best);
  }
  return best;
}

namespace {

bool IntersectRayPlane(const QVector3D& origin, const QVector3D& direction,
                       const QVector3D& plane_point,
                       const QVector3D& plane_normal, QVector3D* hit) {
  const float denom = QVector3D::dotProduct(direction, plane_normal);
  if (std::abs(denom) < 1e-8f) {
    return false;
  }
  const float t =
      QVector3D::dotProduct(plane_point - origin, plane_normal) / denom;
  if (t < 0.f) {
    return false;
  }
  if (hit != nullptr) {
    *hit = origin + direction * t;
  }
  return true;
}

bool ClosestPointOnAxis(const QVector3D& ray_o, const QVector3D& ray_d,
                        const QVector3D& axis_o, const QVector3D& axis_d,
                        QVector3D* hit) {
  const QVector3D w = ray_o - axis_o;
  const float a = QVector3D::dotProduct(ray_d, ray_d);
  const float b = QVector3D::dotProduct(ray_d, axis_d);
  const float c = QVector3D::dotProduct(axis_d, axis_d);
  const float d = QVector3D::dotProduct(ray_d, w);
  const float e = QVector3D::dotProduct(axis_d, w);
  const float denom = a * c - b * b;
  float s = 0.f;
  if (std::abs(denom) > 1e-8f) {
    s = (a * e - b * d) / denom;
  }
  if (hit != nullptr) {
    *hit = axis_o + axis_d * s;
  }
  return true;
}

QQuaternion RotationFromMatrix(const QMatrix4x4& m) {
  const QVector3D x = m.column(0).toVector3D().normalized();
  const QVector3D y = m.column(1).toVector3D().normalized();
  const QVector3D z = m.column(2).toVector3D().normalized();
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
  return QQuaternion::fromRotationMatrix(rot);
}

/**
 * Fixed-frame translation → header/pose-position frame.
 *
 * Pose.position is expressed in the marker header frame, NOT the oriented
 * body frame. Only the header→fixed TF rotation may transform the delta;
 * marker orientation must not (otherwise free 3D drag feels locked to axes).
 *
 * marker_fixed = R_tf * R_marker, so R_tf = R_marker_fixed * R_marker^{-1}.
 */
QVector3D FixedMotionToPosePosition(const QMatrix4x4& marker_fixed,
                                    const QQuaternion& marker_orientation,
                                    const QVector3D& motion_fixed) {
  const QQuaternion marker_ori = marker_orientation.isNull()
                                     ? QQuaternion(1.f, 0.f, 0.f, 0.f)
                                     : marker_orientation.normalized();
  const QQuaternion fixed_from_header =
      RotationFromMatrix(marker_fixed) * marker_ori.conjugated();
  return fixed_from_header.conjugated().rotatedVector(motion_fixed);
}

}  // namespace

bool InteractiveMarkerRegistry::dragPointFromRay(
    uint32_t interaction_mode, const QMatrix4x4& control_transform,
    const QVector3D& ray_origin, const QVector3D& ray_direction,
    const QVector3D& view_forward, QVector3D* drag_point, bool shift_held) {
  if (drag_point == nullptr || ray_direction.lengthSquared() < 1e-12f) {
    return false;
  }
  const QVector3D pivot = control_transform.map(QVector3D(0.f, 0.f, 0.f));
  const QVector3D axis = AxisFromTransform(control_transform, 0);
  const QVector3D dir = ray_direction.normalized();
  switch (interaction_mode) {
    case kMoveAxis:
      return ClosestPointOnAxis(ray_origin, dir, pivot, axis, drag_point);
    case kMovePlane:
    case kMoveRotate:
    case kRotateAxis:
      return IntersectRayPlane(ray_origin, dir, pivot, axis, drag_point);
    case kRotate3d:
    case kMoveRotate3d:
    case kMove3d:
    default: {
      // Shift: free vertical motion along world +Z (any view angle).
      if (shift_held && interaction_mode != kRotate3d) {
        return ClosestPointOnAxis(ray_origin, dir, pivot, QVector3D(0.f, 0.f, 1.f),
                                  drag_point);
      }
      // Default: drag in the view plane → free motion in camera X/Y
      // (combine with Shift-Z for full 3D / 任意方向).
      QVector3D normal = view_forward;
      if (normal.lengthSquared() < 1e-12f) {
        normal = dir;
      }
      return IntersectRayPlane(ray_origin, dir, pivot, normal.normalized(),
                               drag_point);
    }
  }
}

automsgs::msgs::geometry_msgs::Pose InteractiveMarkerRegistry::draggedPose(
    const automsgs::msgs::geometry_msgs::Pose& initial_pose,
    const QMatrix4x4& marker_fixed_at_start,
    const QMatrix4x4& control_transform, uint32_t interaction_mode,
    const QVector3D& initial_drag_point, const QVector3D& current_drag_point,
    bool shift_held, bool ctrl_held) {
  automsgs::msgs::geometry_msgs::Pose pose = initial_pose;
  const QVector3D initial_pos = PosePosition(initial_pose);
  const QQuaternion initial_ori = PoseOrientation(initial_pose);
  const QVector3D delta_fixed = current_drag_point - initial_drag_point;
  const QVector3D pivot_fixed =
      marker_fixed_at_start.map(QVector3D(0.f, 0.f, 0.f));
  const QQuaternion marker_rot = RotationFromMatrix(marker_fixed_at_start);

  const auto apply_fixed_translation = [&](const QVector3D& motion_fixed) {
    SetPosePosition(&pose,
                    initial_pos + FixedMotionToPosePosition(
                                      marker_fixed_at_start, initial_ori,
                                      motion_fixed));
  };

  const auto apply_view_rotation = [&]() {
    const QVector3D v0 = initial_drag_point - pivot_fixed;
    const QVector3D v1 = current_drag_point - pivot_fixed;
    const QVector3D axis = QVector3D::crossProduct(v0, v1);
    if (axis.lengthSquared() > 1e-6f && v0.lengthSquared() > 1e-6f &&
        v1.lengthSquared() > 1e-6f) {
      const float dot = std::clamp(
          QVector3D::dotProduct(v0.normalized(), v1.normalized()), -1.f, 1.f);
      const float angle = std::acos(dot);
      const QQuaternion rot_fixed = QQuaternion::fromAxisAndAngle(
          axis.normalized(), qRadiansToDegrees(angle));
      const QQuaternion rot_pose =
          marker_rot.conjugated() * rot_fixed * marker_rot;
      SetPoseOrientation(&pose, rot_pose * initial_ori);
    }
  };

  switch (interaction_mode) {
    case kMoveAxis: {
      const QVector3D axis = AxisFromTransform(control_transform, 0);
      apply_fixed_translation(axis * QVector3D::dotProduct(delta_fixed, axis));
      break;
    }
    case kMovePlane:
    case kMoveRotate: {
      const QVector3D axis_x = AxisFromTransform(control_transform, 0);
      const QVector3D in_plane =
          delta_fixed - axis_x * QVector3D::dotProduct(delta_fixed, axis_x);
      apply_fixed_translation(in_plane);
      if (interaction_mode == kMoveRotate) {
        const QVector3D v0 = initial_drag_point - pivot_fixed;
        const QVector3D v1 = current_drag_point - pivot_fixed;
        if (v0.lengthSquared() > 1e-6f && v1.lengthSquared() > 1e-6f) {
          const QVector3D v0n = v0.normalized();
          const QVector3D v1n = v1.normalized();
          const float dot =
              std::clamp(QVector3D::dotProduct(v0n, v1n), -1.f, 1.f);
          float angle = std::acos(dot);
          if (QVector3D::dotProduct(QVector3D::crossProduct(v0n, v1n), axis_x) <
              0.f) {
            angle = -angle;
          }
          const QQuaternion rot_fixed =
              QQuaternion::fromAxisAndAngle(axis_x, qRadiansToDegrees(angle));
          const QQuaternion rot_pose = marker_rot.conjugated() * rot_fixed *
                                       marker_rot;
          SetPoseOrientation(&pose, rot_pose * initial_ori);
        }
      }
      break;
    }
    case kRotateAxis: {
      const QVector3D axis = AxisFromTransform(control_transform, 0);
      const QVector3D v0 = initial_drag_point - pivot_fixed;
      const QVector3D v1 = current_drag_point - pivot_fixed;
      const QVector3D v0p = v0 - axis * QVector3D::dotProduct(v0, axis);
      const QVector3D v1p = v1 - axis * QVector3D::dotProduct(v1, axis);
      if (v0p.lengthSquared() > 1e-6f && v1p.lengthSquared() > 1e-6f) {
        const QVector3D v0n = v0p.normalized();
        const QVector3D v1n = v1p.normalized();
        const float dot =
            std::clamp(QVector3D::dotProduct(v0n, v1n), -1.f, 1.f);
        float angle = std::acos(dot);
        if (QVector3D::dotProduct(QVector3D::crossProduct(v0n, v1n), axis) <
            0.f) {
          angle = -angle;
        }
        const QQuaternion rot_fixed =
            QQuaternion::fromAxisAndAngle(axis, qRadiansToDegrees(angle));
        const QQuaternion rot_pose =
            marker_rot.conjugated() * rot_fixed * marker_rot;
        SetPoseOrientation(&pose, rot_pose * initial_ori);
      }
      break;
    }
    case kRotate3d:
      apply_view_rotation();
      break;
    case kMoveRotate3d: {
      // RViz: view-plane drag = free lateral move; Shift = world Z; Ctrl = rotate.
      if (ctrl_held) {
        apply_view_rotation();
      } else {
        // delta_fixed is already constrained (view plane or Z-axis) by
        // dragPointFromRay — apply full 3D delta for 任意方向移动.
        apply_fixed_translation(delta_fixed);
      }
      break;
    }
    case kMove3d:
    default:
      apply_fixed_translation(delta_fixed);
      break;
  }
  return pose;
}

InteractiveMarkerHighlight::Kind InteractiveMarkerRegistry::highlightKind(
    uint32_t interaction_mode, bool ctrl_held) {
  switch (interaction_mode) {
    case kMoveAxis:
    case kMovePlane:
    case kMove3d:
      return InteractiveMarkerHighlight::Kind::kTranslate;
    case kRotateAxis:
    case kRotate3d:
      return InteractiveMarkerHighlight::Kind::kRotate;
    case kMoveRotate:
      return InteractiveMarkerHighlight::Kind::kBoth;
    case kMoveRotate3d:
      return ctrl_held ? InteractiveMarkerHighlight::Kind::kRotate
                       : InteractiveMarkerHighlight::Kind::kTranslate;
    default:
      return InteractiveMarkerHighlight::Kind::kNone;
  }
}

}  // namespace display
}  // namespace autoviz

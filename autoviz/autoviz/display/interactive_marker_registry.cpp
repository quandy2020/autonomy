/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/interactive_marker_registry.hpp"

#include <algorithm>
#include <cmath>

#include <QMatrix4x4>
#include <QQuaternion>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autoviz/display/marker_draw_utils.hpp"
#include "autoviz/display/transform_utils.hpp"

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
  return markerTransformInFixedFrame(pose_marker, context);
}

QMatrix4x4 ControlTransform(
    const automsgs::msgs::visualization_msgs::InteractiveMarker& marker,
    const automsgs::msgs::visualization_msgs::InteractiveMarkerControl&
        control,
    common::DisplayContext* context) {
  QMatrix4x4 control_transform = MarkerPoseMatrix(marker, context);
  const auto& q = control.orientation();
  if (q.w() != 0.0 || q.x() != 0.0 || q.y() != 0.0 || q.z() != 0.0) {
    QMatrix4x4 local;
    local.setToIdentity();
    local.rotate(QQuaternion(static_cast<float>(q.w()), static_cast<float>(q.x()),
                             static_cast<float>(q.y()),
                             static_cast<float>(q.z())));
    control_transform = control_transform * local;
  }
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

void CollectControlPickSamples(
    const QMatrix4x4& control_transform, float scale, uint32_t interaction_mode,
    std::vector<QVector3D>* samples) {
  const float s = std::max(scale, 0.05f);
  switch (interaction_mode) {
    case kMoveAxis: {
      samples->push_back(control_transform.map(QVector3D(0.f, 0.f, 0.f)));
      samples->push_back(control_transform.map(QVector3D(s, 0.f, 0.f)));
      break;
    }
    case kMovePlane:
    case kMoveRotate: {
      samples->push_back(control_transform.map(QVector3D(0.f, 0.f, 0.f)));
      samples->push_back(control_transform.map(QVector3D(0.f, -s, -s)));
      samples->push_back(control_transform.map(QVector3D(0.f, s, s)));
      break;
    }
    case kRotateAxis:
    case kRotate3d: {
      const int segments = 8;
      for (int i = 0; i <= segments; ++i) {
        const float angle = static_cast<float>(i) * 6.2831853f /
                            static_cast<float>(segments);
        samples->push_back(control_transform.map(
            QVector3D(0.f, s * std::cos(angle), s * std::sin(angle))));
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
    QMatrix4x4 control_transform = ControlTransform(marker, control, context);
    if (control.markers_size() > 0) {
      std::map<MarkerKey, StoredMarker> control_markers;
      for (const auto& embedded : control.markers()) {
        upsertMarker(embedded, &control_markers);
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
      candidate.set_frame_id(marker.header().frame_id().empty()
                               ? context->fixed_frame
                               : marker.header().frame_id());
      *candidate.mutable_position() = best_sample;
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
          marker_best.set_frame_id(marker.header().frame_id().empty()
                                     ? context->fixed_frame
                                     : marker.header().frame_id());
          *marker_best.mutable_position() = marker_center;
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

automsgs::msgs::geometry_msgs::Pose InteractiveMarkerRegistry::draggedPose(
    const automsgs::msgs::geometry_msgs::Pose& initial_pose,
    const QMatrix4x4& control_transform, uint32_t interaction_mode,
    const QVector3D& initial_ground, const QVector3D& current_ground,
    bool shift_held) {
  automsgs::msgs::geometry_msgs::Pose pose = initial_pose;
  const QVector3D initial_pos = PosePosition(initial_pose);
  const QQuaternion initial_ori = PoseOrientation(initial_pose);
  const QVector3D delta = current_ground - initial_ground;

  const auto translate_free = [&]() {
    SetPosePosition(&pose, initial_pos + delta);
  };

  switch (interaction_mode) {
    case kMoveAxis: {
      const QVector3D axis = AxisFromTransform(control_transform, 0);
      const float along = QVector3D::dotProduct(delta, axis);
      SetPosePosition(&pose, initial_pos + axis * along);
      break;
    }
    case kMovePlane:
    case kMoveRotate: {
      const QVector3D axis_x = AxisFromTransform(control_transform, 0);
      const QVector3D in_plane =
          delta - axis_x * QVector3D::dotProduct(delta, axis_x);
      SetPosePosition(&pose, initial_pos + in_plane);
      if (interaction_mode == kMoveRotate) {
        const QVector3D pivot = initial_pos;
        const QVector3D v0 = initial_ground - pivot;
        const QVector3D v1 = current_ground - pivot;
        if (v0.lengthSquared() > 1e-6f && v1.lengthSquared() > 1e-6f) {
          const QVector3D v0n = v0.normalized();
          const QVector3D v1n = v1.normalized();
          const float dot = std::clamp(QVector3D::dotProduct(v0n, v1n), -1.f, 1.f);
          float angle = std::acos(dot);
          if (QVector3D::dotProduct(QVector3D::crossProduct(v0n, v1n), axis_x) <
              0.f) {
            angle = -angle;
          }
          const QQuaternion rot =
              QQuaternion::fromAxisAndAngle(axis_x, qRadiansToDegrees(angle));
          SetPosePosition(&pose, pivot + rot.rotatedVector(initial_pos - pivot));
          SetPoseOrientation(&pose, rot * initial_ori);
        }
      }
      break;
    }
    case kRotateAxis: {
      const QVector3D axis = AxisFromTransform(control_transform, 0);
      const QVector3D pivot = initial_pos;
      const QVector3D v0 = initial_ground - pivot;
      const QVector3D v1 = current_ground - pivot;
      if (v0.lengthSquared() > 1e-6f && v1.lengthSquared() > 1e-6f) {
        const QVector3D v0p = v0 - axis * QVector3D::dotProduct(v0, axis);
        const QVector3D v1p = v1 - axis * QVector3D::dotProduct(v1, axis);
        if (v0p.lengthSquared() > 1e-6f && v1p.lengthSquared() > 1e-6f) {
          const QVector3D v0n = v0p.normalized();
          const QVector3D v1n = v1p.normalized();
          const float dot = std::clamp(QVector3D::dotProduct(v0n, v1n), -1.f, 1.f);
          float angle = std::acos(dot);
          if (QVector3D::dotProduct(QVector3D::crossProduct(v0n, v1n), axis) <
              0.f) {
            angle = -angle;
          }
          const QQuaternion rot =
              QQuaternion::fromAxisAndAngle(axis, qRadiansToDegrees(angle));
          SetPosePosition(&pose, pivot + rot.rotatedVector(initial_pos - pivot));
          SetPoseOrientation(&pose, rot * initial_ori);
        }
      }
      break;
    }
    case kRotate3d:
    case kMoveRotate3d: {
      if (shift_held || interaction_mode == kRotate3d) {
        const QVector3D pivot = initial_pos;
        const QVector3D v0 = initial_ground - pivot;
        const QVector3D v1 = current_ground - pivot;
        const QVector3D axis = QVector3D::crossProduct(v0, v1);
        if (axis.lengthSquared() > 1e-6f && v0.lengthSquared() > 1e-6f &&
            v1.lengthSquared() > 1e-6f) {
          const float dot = std::clamp(QVector3D::dotProduct(v0.normalized(),
                                                             v1.normalized()),
                                      -1.f, 1.f);
          float angle = std::acos(dot);
          const QQuaternion rot = QQuaternion::fromAxisAndAngle(
              axis.normalized(), qRadiansToDegrees(angle));
          SetPosePosition(&pose, pivot + rot.rotatedVector(initial_pos - pivot));
          SetPoseOrientation(&pose, rot * initial_ori);
        }
      } else {
        translate_free();
      }
      break;
    }
    case kMove3d:
    default:
      translate_free();
      break;
  }
  return pose;
}

}  // namespace display
}  // namespace autoviz

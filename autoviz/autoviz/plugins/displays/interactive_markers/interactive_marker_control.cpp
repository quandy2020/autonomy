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
#include "autoviz/rendering/view_controller.hpp"

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

/** interactive_markers::assignDefaultColor — RGB from control X axis. */
void AssignDefaultColor(automsgs::msgs::visualization_msgs::Marker* marker,
                        const QQuaternion& control_orientation) {
  if (marker == nullptr) {
    return;
  }
  const QVector3D axis =
      control_orientation.rotatedVector(QVector3D(1.f, 0.f, 0.f));
  const float x = std::abs(axis.x());
  const float y = std::abs(axis.y());
  const float z = std::abs(axis.z());
  const float max_xyz = std::max(x, std::max(y, z));
  marker->mutable_color()->set_r(max_xyz > 0.f ? x / max_xyz : 1.f);
  marker->mutable_color()->set_g(max_xyz > 0.f ? y / max_xyz : 0.f);
  marker->mutable_color()->set_b(max_xyz > 0.f ? z / max_xyz : 0.f);
  marker->mutable_color()->set_a(0.5f);
}

int NextSyntheticMarkerId() {
  static int id = 0;
  return id++;
}

/** interactive_markers::makeArrow */
void MakeArrow(float marker_scale,
               const QQuaternion& control_orientation, float pos,
               std::vector<automsgs::msgs::visualization_msgs::Marker>* out) {
  if (out == nullptr) {
    return;
  }
  automsgs::msgs::visualization_msgs::Marker marker;
  marker.set_id(NextSyntheticMarkerId());
  marker.mutable_pose()->mutable_orientation()->set_w(
      control_orientation.scalar());
  marker.mutable_pose()->mutable_orientation()->set_x(control_orientation.x());
  marker.mutable_pose()->mutable_orientation()->set_y(control_orientation.y());
  marker.mutable_pose()->mutable_orientation()->set_z(control_orientation.z());
  marker.set_type(automsgs::msgs::visualization_msgs::Marker::ARROW);
  marker.mutable_scale()->set_x(marker_scale * 0.15);
  marker.mutable_scale()->set_y(marker_scale * 0.25);
  marker.mutable_scale()->set_z(marker_scale * 0.2);
  AssignDefaultColor(&marker, control_orientation);

  const float dist = std::abs(pos);
  const float dir = pos > 0.f ? 1.f : -1.f;
  const float inner = 0.5f * dist;
  const float outer = inner + 0.4f;
  auto* p0 = marker.add_points();
  p0->set_x(dir * marker_scale * inner);
  auto* p1 = marker.add_points();
  p1->set_x(dir * marker_scale * outer);
  out->push_back(std::move(marker));
}

/** interactive_markers::makeDisc (width default 0.3) */
void MakeDisc(float marker_scale, const QQuaternion& control_orientation,
              uint32_t interaction_mode, float width,
              std::vector<automsgs::msgs::visualization_msgs::Marker>* out) {
  if (out == nullptr) {
    return;
  }
  automsgs::msgs::visualization_msgs::Marker marker;
  marker.set_id(NextSyntheticMarkerId());
  marker.mutable_pose()->mutable_orientation()->set_w(
      control_orientation.scalar());
  marker.mutable_pose()->mutable_orientation()->set_x(control_orientation.x());
  marker.mutable_pose()->mutable_orientation()->set_y(control_orientation.y());
  marker.mutable_pose()->mutable_orientation()->set_z(control_orientation.z());
  marker.set_type(automsgs::msgs::visualization_msgs::Marker::TRIANGLE_LIST);
  marker.mutable_scale()->set_x(marker_scale);
  marker.mutable_scale()->set_y(marker_scale);
  marker.mutable_scale()->set_z(marker_scale);
  AssignDefaultColor(&marker, control_orientation);

  constexpr int kSteps = 36;
  std::vector<QVector3D> circle1;
  std::vector<QVector3D> circle2;
  circle1.reserve(kSteps);
  circle2.reserve(kSteps);
  for (int i = 0; i < kSteps; ++i) {
    const float a =
        static_cast<float>(i) / static_cast<float>(kSteps) * 6.2831853f;
    const QVector3D v1(0.f, 0.5f * std::cos(a), 0.5f * std::sin(a));
    circle1.push_back(v1);
    circle2.push_back((1.f + width) * v1);
  }

  const auto& base = marker.color();
  auto add_tri = [&](const QVector3D& a, const QVector3D& b, const QVector3D& c,
                     float r, float g, float bl, float al) {
    auto* p0 = marker.add_points();
    p0->set_x(a.x());
    p0->set_y(a.y());
    p0->set_z(a.z());
    auto* p1 = marker.add_points();
    p1->set_x(b.x());
    p1->set_y(b.y());
    p1->set_z(b.z());
    auto* p2 = marker.add_points();
    p2->set_x(c.x());
    p2->set_y(c.y());
    p2->set_z(c.z());
    auto* col = marker.add_colors();
    col->set_r(r);
    col->set_g(g);
    col->set_b(bl);
    col->set_a(al);
  };

  // ROS tools.cpp leave color.a=1 on stripe verts; RViz still paints discs
  // translucent via marker.color.a (0.5). Use base.a() so rings match the
  // classic semi-transparent striped 6-DoF look.
  const float disc_a = base.a() > 0.f ? base.a() : 0.5f;
  if (interaction_mode == kRotateAxis) {
    // Full annulus sectors (not the ROS zigzag) so winding stays consistent
    // around the ring; alternating shade keeps the classic striped look.
    for (int i = 0; i < kSteps; ++i) {
      const int i1 = i;
      const int i2 = (i + 1) % kSteps;
      const float t = 0.6f + 0.4f * static_cast<float>(i % 2);
      const float r = base.r() * t;
      const float g = base.g() * t;
      const float b = base.b() * t;
      add_tri(circle1[i1], circle2[i1], circle1[i2], r, g, b, disc_a);
      add_tri(circle2[i1], circle2[i2], circle1[i2], r, g, b, disc_a);
    }
  } else if (interaction_mode == kMoveRotate) {
    for (int i = 0; i < kSteps - 1; i += 2) {
      const int i1 = i;
      const int i2 = (i + 1) % kSteps;
      const int i3 = (i + 2) % kSteps;
      add_tri(circle1[i1], circle2[i2], circle1[i2], base.r() * 0.6f,
              base.g() * 0.6f, base.b() * 0.6f, disc_a);
      add_tri(circle1[i2], circle2[i2], circle1[i3], base.r() * 0.6f,
              base.g() * 0.6f, base.b() * 0.6f, disc_a);
      add_tri(circle2[i1], circle2[i2], circle1[i1], base.r(), base.g(),
              base.b(), disc_a);
      add_tri(circle2[i2], circle2[i3], circle1[i3], base.r(), base.g(),
              base.b(), disc_a);
    }
  } else {
    for (int i = 0; i < kSteps; ++i) {
      const int i1 = i;
      const int i2 = (i + 1) % kSteps;
      add_tri(circle1[i1], circle2[i1], circle1[i2], base.r(), base.g(),
              base.b(), disc_a);
      add_tri(circle2[i1], circle2[i2], circle1[i2], base.r(), base.g(),
              base.b(), disc_a);
    }
  }
  out->push_back(std::move(marker));
}

/**
 * @brief Fills default handles when a control has no markers (ROS autoComplete).
 */
void AutoCompleteMarkers(
    float marker_scale, const QQuaternion& control_orientation,
    uint32_t interaction_mode,
    std::vector<automsgs::msgs::visualization_msgs::Marker>* markers) {
  if (markers == nullptr || !markers->empty()) {
    return;
  }
  switch (interaction_mode) {
    case kMoveAxis:
      MakeArrow(marker_scale, control_orientation, 1.f, markers);
      MakeArrow(marker_scale, control_orientation, -1.f, markers);
      break;
    case kMovePlane:
    case kRotateAxis:
    case kMoveRotate:
      MakeDisc(marker_scale, control_orientation, interaction_mode, 0.3f,
               markers);
      break;
    default:
      break;
  }
}

void AutoCompleteControl(
    float marker_scale,
    automsgs::msgs::visualization_msgs::InteractiveMarkerControl* control) {
  if (control == nullptr || control->markers_size() > 0) {
    return;
  }
  const auto& q = control->orientation();
  QQuaternion ori(1.f, 0.f, 0.f, 0.f);
  if (q.w() != 0.0 || q.x() != 0.0 || q.y() != 0.0 || q.z() != 0.0) {
    ori = QQuaternion(static_cast<float>(q.w()), static_cast<float>(q.x()),
                      static_cast<float>(q.y()), static_cast<float>(q.z()))
              .normalized();
  } else {
    control->mutable_orientation()->set_w(1.0);
  }
  std::vector<automsgs::msgs::visualization_msgs::Marker> markers;
  AutoCompleteMarkers(marker_scale, ori,
                      static_cast<uint32_t>(control->interaction_mode()),
                      &markers);
  for (auto& m : markers) {
    *control->add_markers() = std::move(m);
  }
}

/**
 * @brief RViz updateControlOrientationForViewFacing (rotation_=0).
 */
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
    case kMove3d: {
      const QVector3D center = parent_transform.map(QVector3D(0.f, 0.f, 0.f));
      scene.addPoint(center, color);
      break;
    }
    case kInteractionButton:
      scene.addPoint(parent_transform.map(QVector3D(0.f, 0.f, 0.f)), color);
      break;
    default:
      break;
  }
}

}  // namespace

void autoCompleteInteractiveMarker(
    automsgs::msgs::visualization_msgs::InteractiveMarker* marker) {
  if (marker == nullptr) {
    return;
  }
  if (marker->scale() == 0.0) {
    marker->set_scale(1.0);
  }
  auto* q = marker->mutable_pose()->mutable_orientation();
  if (q->w() == 0.0 && q->x() == 0.0 && q->y() == 0.0 && q->z() == 0.0) {
    q->set_w(1.0);
  }
  const float scale = static_cast<float>(marker->scale());
  for (int i = 0; i < marker->controls_size(); ++i) {
    AutoCompleteControl(scale, marker->mutable_controls(i));
  }
}

InteractiveMarkerControl::InteractiveMarkerControl(
    common::DisplayContext* context, InteractiveMarker* parent)
    : context_(context), parent_(parent) {
  control_orientation_ = QQuaternion(1.f, 0.f, 0.f, 0.f);
  parent_orientation_ = QQuaternion(1.f, 0.f, 0.f, 0.f);
  control_transform_.setToIdentity();
  markers_transform_.setToIdentity();
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
                    static_cast<float>(q.y()), static_cast<float>(q.z()))
            .normalized();
  }

  markers_.clear();
  markers_.reserve(static_cast<std::size_t>(message.markers_size()));
  for (const auto& marker : message.markers()) {
    markers_.push_back(marker);
  }

  if (parent_ != nullptr) {
    parent_position_ = parent_->getPosition();
    parent_orientation_ = parent_->getOrientation();
    AutoCompleteMarkers(parent_->getSize(), control_orientation_,
                        static_cast<uint32_t>(interaction_mode_), &markers_);
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

void InteractiveMarkerControl::setHighlight(ControlHighlight highlight,
                                            HighlightKind kind) {
  highlight_ = static_cast<float>(highlight) * 0.1f;
  highlight_kind_ = kind;
}

InteractiveMarkerControl::HighlightKind
InteractiveMarkerControl::resolvedHighlightKind() const {
  if (highlight_kind_ != HighlightKind::kAuto) {
    return highlight_kind_;
  }
  switch (static_cast<uint32_t>(interaction_mode_)) {
    case kMoveAxis:
    case kMovePlane:
    case kMove3d:
      return HighlightKind::kTranslate;
    case kRotateAxis:
    case kRotate3d:
      return HighlightKind::kRotate;
    case kMoveRotate:
    case kMoveRotate3d:
      return HighlightKind::kBoth;
    default:
      return HighlightKind::kAuto;
  }
}

void InteractiveMarkerControl::rebuildControlTransform() {
  using Mode = automsgs::msgs::visualization_msgs::InteractiveMarkerControl;

  // Parent pose in fixed frame (RViz reference_node_ * marker pose).
  QMatrix4x4 parent_tf = ComposePose(parent_position_, parent_orientation_);
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
    parent_tf = markerPoseTransformInFixedFrame(pose_marker, context_);
  }
  const QVector3D parent_pos = parent_tf.map(QVector3D(0.f, 0.f, 0.f));
  const QQuaternion parent_ori = RotationFromMatrix(parent_tf);

  QQuaternion control_frame_ori = parent_ori;
  QQuaternion markers_ori = parent_ori;

  if (orientation_mode_ == Mode::FIXED) {
    // RViz: identity in reference frame when rotation_=0; marker poses carry
    // control orientation (autoComplete). Interaction axis uses control ori.
    control_frame_ori = QQuaternion(1.f, 0.f, 0.f, 0.f);
    markers_ori = QQuaternion(1.f, 0.f, 0.f, 0.f);
  } else if (orientation_mode_ == Mode::VIEW_FACING) {
    QVector3D cam_forward(0.f, 0.f, -1.f);
    QVector3D cam_up(0.f, 0.f, 1.f);
    if (context_ != nullptr) {
      if (context_->view_controller != nullptr) {
        const QMatrix4x4 inv = context_->view_controller->viewMatrix().inverted();
        cam_forward =
            (inv * QVector4D(0.f, 0.f, -1.f, 0.f)).toVector3D().normalized();
        cam_up = (inv * QVector4D(0.f, 1.f, 0.f, 0.f)).toVector3D().normalized();
      } else if (context_->has_view_matrices) {
        const QMatrix4x4 inv = context_->view_matrix.inverted();
        cam_forward =
            (inv * QVector4D(0.f, 0.f, -1.f, 0.f)).toVector3D().normalized();
        cam_up = (inv * QVector4D(0.f, 1.f, 0.f, 0.f)).toVector3D().normalized();
      }
    }
    const QQuaternion view_ori =
        ViewFacingOrientation(control_orientation_, cam_forward, cam_up);
    control_frame_ori = view_ori;
    markers_ori =
        independent_marker_orientation_ ? parent_ori : view_ori;
  } else {
    // INHERIT: markers_node = parent; interaction axis = parent * control_ori.
    control_frame_ori = parent_ori;
    markers_ori = parent_ori;
  }

  // Drag / pick frame: RViz control_frame_ori * control_orientation_.xAxis().
  control_transform_ =
      ComposePose(parent_pos, control_frame_ori * control_orientation_);
  markers_transform_ = ComposePose(parent_pos, markers_ori);
}

void InteractiveMarkerControl::draw(
    rendering::SceneOverlay& scene,
    const common::DisplayPropertyMap& properties,
    const std::string& draw_prefix) const {
  if (!visible_ && !always_visible_) {
    return;
  }
  const float scale = parent_ != nullptr ? parent_->getSize() : 0.2f;
  const HighlightKind kind = resolvedHighlightKind();
  // Translate → cyan, rotate → amber (readable against RGB axis handles).
  const QColor translate_tint(80, 220, 255);
  const QColor rotate_tint(255, 200, 60);
  QColor color(255, 220, 80);
  if (highlight_ > 0.f) {
    if (kind == HighlightKind::kRotate) {
      color = rotate_tint;
    } else if (kind == HighlightKind::kTranslate) {
      color = translate_tint;
    } else if (kind == HighlightKind::kBoth) {
      color = QColor(200, 255, 160);
    }
    color = color.lighter(100 + static_cast<int>(highlight_ * 120.f));
  }

  const auto boost_color =
      [&](automsgs::msgs::std_msgs::ColorRGBA* c, const QColor& tint) {
        if (c == nullptr || highlight_ <= 0.f) {
          return;
        }
        const float t = std::clamp(highlight_ * 1.4f, 0.f, 0.85f);
        c->set_r(std::min(1.f, c->r() * (1.f - t) + tint.redF() * t + highlight_));
        c->set_g(std::min(1.f, c->g() * (1.f - t) + tint.greenF() * t + highlight_));
        c->set_b(std::min(1.f, c->b() * (1.f - t) + tint.blueF() * t + highlight_));
        c->set_a(std::min(1.f, std::max(c->a(), 0.55f + highlight_)));
      };

  if (!markers_.empty()) {
    // RViz markers_node_: marker poses are relative to the interactive marker
    // (not the control frame). autoComplete puts control.orientation on the
    // marker pose so arrows/discs align with the interaction axis.
    std::map<MarkerKey, StoredMarker> control_markers;
    for (const auto& embedded : markers_) {
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
      const QVector3D pos = markers_transform_.map(local_pos);
      const QQuaternion markers_ori = RotationFromMatrix(markers_transform_);
      const QQuaternion ori = markers_ori * local_ori;

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
      if (highlight_ > 0.f) {
        const QColor tint = (kind == HighlightKind::kRotate) ? rotate_tint
                            : (kind == HighlightKind::kTranslate)
                                  ? translate_tint
                                  : QColor(220, 255, 180);
        boost_color(transformed.mutable_color(), tint);
        for (int i = 0; i < transformed.colors_size(); ++i) {
          boost_color(transformed.mutable_colors(i), tint);
        }
      }
      upsertMarker(transformed, &control_markers);
    }
    drawStoredMarkers(scene, context_, properties, control_markers,
                      draw_prefix);
  } else {
    DrawDefaultControlGeometry(scene, control_transform_, scale,
                               static_cast<uint32_t>(interaction_mode_), color);
  }

  const QVector3D origin = control_transform_.map(QVector3D(0.f, 0.f, 0.f));
  const QVector3D axis = RotationFromMatrix(control_transform_)
                             .rotatedVector(QVector3D(1.f, 0.f, 0.f));
  const bool show_aids =
      (show_visual_aids_ && isInteractive()) || highlight_ > 0.f;
  if (show_aids) {
    const bool want_axis =
        kind == HighlightKind::kTranslate || kind == HighlightKind::kBoth ||
        (highlight_ <= 0.f && show_visual_aids_);
    const bool want_ring =
        kind == HighlightKind::kRotate || kind == HighlightKind::kBoth;
    if (want_axis) {
      const QColor axis_color =
          highlight_ > 0.f ? QColor(80, 230, 255, 200) : QColor(255, 255, 255, 80);
      scene.addLine(origin - axis * (highlight_ > 0.f ? 2.5f * scale : 1000.f),
                    origin + axis * (highlight_ > 0.f ? 2.5f * scale : 1000.f),
                    axis_color);
    }
    if (want_ring && highlight_ > 0.f) {
      const float radius = 0.65f * std::max(scale, 0.05f);
      constexpr int kSegments = 48;
      QVector3D prev =
          control_transform_.map(QVector3D(0.f, radius, 0.f));
      for (int i = 1; i <= kSegments; ++i) {
        const float angle =
            static_cast<float>(i) * 6.2831853f / static_cast<float>(kSegments);
        const QVector3D next = control_transform_.map(
            QVector3D(0.f, radius * std::cos(angle), radius * std::sin(angle)));
        scene.addLine(prev, next, QColor(255, 200, 60, 230));
        prev = next;
      }
    }
  }
}

void InteractiveMarkerControl::collectPickSamples(
    float marker_scale, std::vector<QVector3D>* samples) const {
  if (samples == nullptr || !isInteractive() ||
      interaction_mode_ == static_cast<int>(kInteractionMenu)) {
    return;
  }
  // Match registry CollectControlPickSamples (MakeArrow / MakeDisc radii).
  const float s = std::max(marker_scale, 0.05f);
  switch (static_cast<uint32_t>(interaction_mode_)) {
    case kMoveAxis:
      for (float dir : {1.f, -1.f}) {
        samples->push_back(
            control_transform_.map(QVector3D(dir * 0.5f * s, 0.f, 0.f)));
        samples->push_back(
            control_transform_.map(QVector3D(dir * 0.9f * s, 0.f, 0.f)));
      }
      break;
    case kMovePlane:
    case kMoveRotate:
    case kRotateAxis: {
      constexpr int kSegments = 16;
      for (float radius : {0.5f * s, 0.65f * s}) {
        for (int i = 0; i <= kSegments; ++i) {
          const float angle =
              static_cast<float>(i) * 6.2831853f / static_cast<float>(kSegments);
          samples->push_back(control_transform_.map(
              QVector3D(0.f, radius * std::cos(angle),
                        radius * std::sin(angle))));
        }
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

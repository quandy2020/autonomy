/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/view_controller.hpp"

#include <algorithm>

#include <Qt>
#include <QtMath>

#include "autoviz/rendering/scene_overlay.hpp"
#include "autoviz/common/frame_manager.hpp"
#include "autoviz/common/view_controller_registry.hpp"

namespace autoviz {
namespace rendering {
namespace {

constexpr float kOrbitRotationSpeed = 0.005f;
constexpr float kPerspectiveFovYDegrees = 45.f;

QVector3D UnprojectPixel(const QMatrix4x4& inverse_mvp, float ndc_x,
                         float ndc_y) {
  const QVector4D near_point = inverse_mvp * QVector4D(ndc_x, ndc_y, -1.f, 1.f);
  const QVector4D far_point = inverse_mvp * QVector4D(ndc_x, ndc_y, 1.f, 1.f);
  const QVector3D near_w =
      QVector3D(near_point.x(), near_point.y(), near_point.z()) / near_point.w();
  const QVector3D far_w =
      QVector3D(far_point.x(), far_point.y(), far_point.z()) / far_point.w();
  return far_w - near_w;
}

bool IntersectGroundPlane(const QVector3D& origin, const QVector3D& direction,
                          QVector3D* hit) {
  // REP-103 / RViz2: ground is the XY plane (z = 0).
  if (std::abs(direction.z()) < 1e-6f) {
    return false;
  }
  const float t = -origin.z() / direction.z();
  if (t < 0.f) {
    return false;
  }
  *hit = origin + direction * t;
  return true;
}

}  // namespace

void ViewController::setType(ViewControllerType type) {
  type_ = type;
  if (type_ == ViewControllerType::kTopDown ||
      type_ == ViewControllerType::kTopDownOrtho) {
    pitch_ = 1.45f;
    if (type_ == ViewControllerType::kTopDownOrtho) {
      yaw_ = 0.f;
    }
  } else if (type_ == ViewControllerType::kXyOrbit) {
    pitch_ = xy_orbit_pitch_;
  }
}

void ViewController::setTypeByName(const QString& name) {
  common::ViewControllerRegistry::instance().applyByName(name, this);
}

QString ViewController::typeName() const {
  switch (type_) {
    case ViewControllerType::kTopDown:
      return QStringLiteral("TopDown");
    case ViewControllerType::kXyOrbit:
      return QStringLiteral("XYOrbit");
    case ViewControllerType::kTopDownOrtho:
      return QStringLiteral("TopDownOrtho");
    case ViewControllerType::kFps:
      return QStringLiteral("FPS");
    case ViewControllerType::kFpsMotion:
      return QStringLiteral("FPSMotion");
    case ViewControllerType::kThirdPersonFollow:
      return QStringLiteral("ThirdPersonFollow");
    case ViewControllerType::kOrbit:
    default:
      return QStringLiteral("Orbit");
  }
}

ViewState ViewController::state() const {
  ViewState state;
  state.type = type_;
  state.near_clip_distance = near_clip_distance_;
  state.invert_z_axis = invert_z_axis_;
  state.yaw = yaw_;
  state.pitch = pitch_;
  state.distance = distance_;
  state.target = target_;
  state.focal_shape_size = focal_shape_size_;
  state.focal_shape_fixed_size = focal_shape_fixed_size_;
  state.fps_position = fps_position_;
  state.fps_yaw = fps_yaw_;
  state.fps_pitch = fps_pitch_;
  state.target_frame = target_frame_;
  return state;
}

void ViewController::setState(const ViewState& state) {
  type_ = state.type;
  near_clip_distance_ = state.near_clip_distance;
  invert_z_axis_ = state.invert_z_axis;
  yaw_ = state.yaw;
  pitch_ = state.pitch;
  distance_ = state.distance;
  target_ = state.target;
  focal_shape_size_ = state.focal_shape_size;
  focal_shape_fixed_size_ = state.focal_shape_fixed_size;
  fps_position_ = state.fps_position;
  fps_yaw_ = state.fps_yaw;
  fps_pitch_ = state.fps_pitch;
  target_frame_ = state.target_frame;
}

QString ViewController::targetFrameDisplay() const {
  if (target_frame_.empty()) {
    return ViewTargetFrameFixedSentinel();
  }
  return QString::fromStdString(target_frame_);
}

void ViewController::setTargetFrame(const QString& frame) {
  const QString trimmed = frame.trimmed();
  if (trimmed.isEmpty() || trimmed == ViewTargetFrameFixedSentinel()) {
    target_frame_.clear();
    return;
  }
  target_frame_ = trimmed.toStdString();
}

bool ViewController::tracksTargetFrame() const { return !target_frame_.empty(); }

QMatrix4x4 ViewController::targetFrameToFixedMatrix() const {
  QMatrix4x4 matrix;
  if (!tracksTargetFrame() || frame_manager_ == nullptr) {
    return matrix;
  }
  QVector3D position;
  QQuaternion orientation;
  if (!frame_manager_->getTransform(target_frame_, &position, &orientation)) {
    return matrix;
  }
  matrix.translate(position);
  matrix.rotate(orientation);
  return matrix;
}

void ViewController::reset() {
  near_clip_distance_ = 0.01f;
  invert_z_axis_ = false;
  yaw_ = 0.785398f;
  pitch_ = 0.785398f;
  distance_ = 10.f;
  target_ = QVector3D(0.f, 0.f, 0.f);
  focal_shape_size_ = 0.05f;
  focal_shape_fixed_size_ = true;
  fps_position_ = QVector3D(0.f, 0.f, 2.f);
  fps_yaw_ = 3.14f;
  fps_pitch_ = 0.f;
  fps_fly_mode_ = false;
}

void ViewController::setNearClipDistance(float value) {
  near_clip_distance_ = std::max(0.001f, value);
}

void ViewController::setInvertZAxis(bool value) { invert_z_axis_ = value; }

void ViewController::setFocalShapeSize(float value) {
  focal_shape_size_ = std::max(0.001f, value);
}

void ViewController::setFocalShapeFixedSize(bool value) {
  focal_shape_fixed_size_ = value;
}

QMatrix4x4 ViewController::projectionMatrix(float aspect_ratio) const {
  QMatrix4x4 projection;
  const float far_plane = 500.f;
  if (type_ == ViewControllerType::kTopDownOrtho) {
    const float half_height = distance_ * 0.5f;
    const float half_width = half_height * aspect_ratio;
    projection.ortho(-half_width, half_width, -half_height, half_height,
                     near_clip_distance_, far_plane);
    return projection;
  }
  projection.perspective(45.f, aspect_ratio, near_clip_distance_, far_plane);
  if (invert_z_axis_) {
    projection(2, 2) *= -1.f;
  }
  return projection;
}

QMatrix4x4 ViewController::orbitViewMatrix() const {
  // RViz2 OrbitViewController (REP-103 Z-up): yaw about Z, pitch elevates Z.
  const float x = distance_ * qCos(yaw_) * qCos(pitch_);
  const float y = distance_ * qSin(yaw_) * qCos(pitch_);
  const float z = distance_ * qSin(pitch_);
  const QVector3D eye = target_ + QVector3D(x, y, z);
  QMatrix4x4 view;
  view.lookAt(eye, target_, QVector3D(0.f, 0.f, 1.f));
  return view;
}

QMatrix4x4 ViewController::topDownViewMatrix() const {
  const float x = distance_ * qCos(yaw_);
  const float y = distance_ * qSin(yaw_);
  const QVector3D eye = target_ + QVector3D(x, y, distance_);
  QMatrix4x4 view;
  view.lookAt(eye, target_, QVector3D(0.f, 1.f, 0.f));
  return view;
}

QMatrix4x4 ViewController::thirdPersonViewMatrix() const {
  const float x = distance_ * qCos(yaw_);
  const float y = distance_ * qSin(yaw_);
  const float height = std::max(2.f, distance_ * 0.35f);
  const QVector3D eye = target_ + QVector3D(x, y, height);
  QMatrix4x4 view;
  view.lookAt(eye, target_, QVector3D(0.f, 0.f, 1.f));
  return view;
}

QMatrix4x4 ViewController::fpsViewMatrix() const {
  const float cx = qCos(fps_pitch_) * qCos(fps_yaw_);
  const float cy = qCos(fps_pitch_) * qSin(fps_yaw_);
  const float cz = qSin(fps_pitch_);
  const QVector3D forward(cx, cy, cz);
  QMatrix4x4 view;
  view.lookAt(fps_position_, fps_position_ + forward, QVector3D(0.f, 0.f, 1.f));
  return view;
}

QMatrix4x4 ViewController::viewMatrix() const {
  QMatrix4x4 view = localViewMatrix();
  if (tracksTargetFrame()) {
    const QMatrix4x4 target_to_fixed = targetFrameToFixedMatrix();
    if (!target_to_fixed.isIdentity()) {
      view = view * target_to_fixed.inverted();
    }
  }
  return view;
}

QMatrix4x4 ViewController::localViewMatrix() const {
  switch (type_) {
    case ViewControllerType::kTopDown:
    case ViewControllerType::kTopDownOrtho:
      return topDownViewMatrix();
    case ViewControllerType::kXyOrbit:
      return orbitViewMatrix();
    case ViewControllerType::kThirdPersonFollow:
      return thirdPersonViewMatrix();
    case ViewControllerType::kFps:
      return fpsViewMatrix();
    case ViewControllerType::kFpsMotion:
      return orbitViewMatrix();
    case ViewControllerType::kOrbit:
    default:
      return orbitViewMatrix();
  }
}

void ViewController::orbitBy(float delta_yaw, float delta_pitch) {
  if (type_ == ViewControllerType::kFps) {
    fps_yaw_ += delta_yaw;
    fps_pitch_ = qBound(-1.4f, fps_pitch_ + delta_pitch, 1.4f);
    return;
  }
  if (type_ == ViewControllerType::kTopDownOrtho) {
    return;
  }
  yaw_ += delta_yaw;
  if (type_ == ViewControllerType::kTopDown ||
      type_ == ViewControllerType::kXyOrbit ||
      type_ == ViewControllerType::kThirdPersonFollow) {
    return;
  }
  pitch_ = qBound(-1.4f, pitch_ + delta_pitch, 1.4f);
}

void ViewController::panBy(float delta_x, float delta_y) {
  const float scale = distance_ * 0.002f;
  const float cos_yaw = qCos(yaw_);
  const float sin_yaw = qSin(yaw_);
  const QVector3D right(cos_yaw, sin_yaw, 0.f);
  const QVector3D forward(-sin_yaw, cos_yaw, 0.f);
  target_ -= right * (delta_x * scale);
  target_ += forward * (delta_y * scale);
  if (type_ == ViewControllerType::kFps) {
    fps_position_ -= right * (delta_x * scale);
    fps_position_ += forward * (delta_y * scale);
  }
}

void ViewController::zoomBy(float delta_distance) {
  zoomCamera(delta_distance);
}

QVector3D ViewController::cameraLocalToWorld(float local_x, float local_y,
                                             float local_z) const {
  const QMatrix4x4 inverse_view = viewMatrix().inverted();
  const QVector3D right = inverse_view.mapVector(QVector3D(1.f, 0.f, 0.f)).normalized();
  const QVector3D up = inverse_view.mapVector(QVector3D(0.f, 1.f, 0.f)).normalized();
  const QVector3D forward =
      inverse_view.mapVector(QVector3D(0.f, 0.f, -1.f)).normalized();
  return right * local_x + up * local_y + forward * local_z;
}

void ViewController::applyFocalDelta(const QVector3D& delta) {
  target_ += delta;
  if (type_ == ViewControllerType::kFps) {
    fps_position_ += delta;
  }
}

void ViewController::rotateCamera(float diff_x, float diff_y) {
  orbitBy(diff_x * kOrbitRotationSpeed, -diff_y * kOrbitRotationSpeed);
}

void ViewController::panFocalPointXY(float diff_x, float diff_y, float aspect_ratio,
                                   int viewport_width, int viewport_height) {
  if (viewport_width <= 0 || viewport_height <= 0) {
    return;
  }
  float move_x = 0.f;
  float move_y = 0.f;
  if (type_ == ViewControllerType::kTopDownOrtho) {
    const float half_height = distance_ * 0.5f;
    const float half_width = half_height * aspect_ratio;
    move_x = -(static_cast<float>(diff_x) / static_cast<float>(viewport_width)) *
             half_width * 2.f;
    move_y = (static_cast<float>(diff_y) / static_cast<float>(viewport_height)) *
             half_height * 2.f;
  } else {
    const float fov_y = qDegreesToRadians(kPerspectiveFovYDegrees);
    const float fov_x = 2.f * qAtan(qTan(fov_y / 2.f) * aspect_ratio);
    move_x = -(static_cast<float>(diff_x) / static_cast<float>(viewport_width)) *
             distance_ * qTan(fov_x / 2.f) * 2.f;
    move_y = (static_cast<float>(diff_y) / static_cast<float>(viewport_height)) *
             distance_ * qTan(fov_y / 2.f) * 2.f;
  }
  applyFocalDelta(cameraLocalToWorld(move_x, move_y, 0.f));
}

void ViewController::panFocalPointZ(float amount) {
  applyFocalDelta(cameraLocalToWorld(0.f, 0.f, amount));
}

void ViewController::zoomCamera(float amount) {
  distance_ = std::max(0.001f, distance_ - amount);
}

ViewController::ViewDragMode ViewController::dragModeForPress(
    Qt::MouseButtons buttons, Qt::KeyboardModifiers modifiers) const {
  const bool shift = modifiers.testFlag(Qt::ShiftModifier);

  switch (type_) {
    case ViewControllerType::kTopDownOrtho:
      if (buttons & Qt::LeftButton) {
        return ViewDragMode::kPanXY;
      }
      return ViewDragMode::kNone;

    case ViewControllerType::kFps:
    case ViewControllerType::kFpsMotion:
      if ((buttons & Qt::LeftButton) && !shift) {
        return ViewDragMode::kOrbit;
      }
      return ViewDragMode::kNone;

    case ViewControllerType::kOrbit:
    case ViewControllerType::kXyOrbit:
    case ViewControllerType::kTopDown:
    case ViewControllerType::kThirdPersonFollow:
    default:
      if ((buttons & Qt::LeftButton) && !shift) {
        return ViewDragMode::kOrbit;
      }
      if ((buttons & Qt::MiddleButton) || ((buttons & Qt::LeftButton) && shift)) {
        return ViewDragMode::kPanXY;
      }
      return ViewDragMode::kNone;
  }
}

bool ViewController::handleMouseEvent(const ViewportMouseEvent& event) {
  switch (event.action) {
    case ViewportMouseEvent::Action::kPress: {
      const ViewDragMode mode =
          dragModeForPress(event.buttons, event.modifiers);
      if (mode == ViewDragMode::kNone) {
        return false;
      }
      drag_mode_ = mode;
      dragging_ = true;
      last_mouse_x_ = event.x();
      last_mouse_y_ = event.y();
      if (type_ != ViewControllerType::kFps &&
          type_ != ViewControllerType::kFpsMotion) {
        setFocalShapeVisible(true);
      }
      return true;
    }
    case ViewportMouseEvent::Action::kRelease:
      dragging_ = false;
      drag_mode_ = ViewDragMode::kNone;
      setFocalShapeVisible(false);
      return true;
    case ViewportMouseEvent::Action::kMove:
      if (!dragging_ || drag_mode_ == ViewDragMode::kNone) {
        return false;
      }
      {
        const int diff_x = event.x() - last_mouse_x_;
        const int diff_y = event.y() - last_mouse_y_;
        last_mouse_x_ = event.x();
        last_mouse_y_ = event.y();
        const float aspect =
            static_cast<float>(event.viewport_width) /
            static_cast<float>(std::max(1, event.viewport_height));
        switch (drag_mode_) {
          case ViewDragMode::kOrbit:
            rotateCamera(static_cast<float>(diff_x),
                         static_cast<float>(diff_y));
            break;
          case ViewDragMode::kPanXY:
            panFocalPointXY(static_cast<float>(diff_x),
                            static_cast<float>(diff_y), aspect,
                            event.viewport_width, event.viewport_height);
            break;
          case ViewDragMode::kZoom:
          case ViewDragMode::kPanZ:
            break;
          case ViewDragMode::kNone:
            break;
        }
      }
      return true;
    case ViewportMouseEvent::Action::kWheel:
      zoomCamera(static_cast<float>(event.wheel_delta) * 0.001f * distance_);
      return true;
  }
  return false;
}

void ViewController::setTarget(const QVector3D& target) {
  const QVector3D delta = target - target_;
  target_ = target;
  if (type_ == ViewControllerType::kFps) {
    fps_position_ += delta;
  }
}

void ViewController::setFpsKey(int key, bool pressed) {
  if (type_ != ViewControllerType::kFps &&
      type_ != ViewControllerType::kFpsMotion) {
    return;
  }
  switch (key) {
    case Qt::Key_W:
      move_forward_ = pressed;
      break;
    case Qt::Key_S:
      move_backward_ = pressed;
      break;
    case Qt::Key_A:
      move_left_ = pressed;
      break;
    case Qt::Key_D:
      move_right_ = pressed;
      break;
    case Qt::Key_Q:
      move_down_ = pressed;
      break;
    case Qt::Key_E:
      move_up_ = pressed;
      break;
    default:
      break;
  }
}

bool ViewController::handleKeyEvent(int key, bool pressed) {
  if (!pressed) {
    return false;
  }
  if (type_ != ViewControllerType::kFps &&
      type_ != ViewControllerType::kFpsMotion) {
    return false;
  }
  switch (key) {
    case Qt::Key_F:
      fps_fly_mode_ = !fps_fly_mode_;
      return true;
    case Qt::Key_R:
      reset();
      return true;
    default:
      return false;
  }
}

void ViewController::tick(float delta_seconds) {
  if (type_ != ViewControllerType::kFps &&
      type_ != ViewControllerType::kFpsMotion) {
    return;
  }
  const float speed = 4.f * delta_seconds;
  if (type_ == ViewControllerType::kFpsMotion) {
    const float heading = yaw_;
    const float cx = qCos(heading);
    const float cy = qSin(heading);
    QVector3D forward(cx, cy, 0.f);
    QVector3D right(-cy, cx, 0.f);
    if (fps_fly_mode_) {
      const float cz = qSin(pitch_);
      const float horiz = qCos(pitch_);
      forward = QVector3D(horiz * cx, horiz * cy, cz);
      right = QVector3D(-cy, cx, 0.f);
    }
    if (move_forward_) {
      target_ += forward * speed;
    }
    if (move_backward_) {
      target_ -= forward * speed;
    }
    if (move_right_) {
      target_ += right * speed;
    }
    if (move_left_) {
      target_ -= right * speed;
    }
    if (fps_fly_mode_) {
      if (move_up_) {
        target_.setZ(target_.z() + speed);
      }
      if (move_down_) {
        target_.setZ(target_.z() - speed);
      }
    }
    return;
  }

  const float cx = qCos(fps_pitch_) * qCos(fps_yaw_);
  const float cy = qCos(fps_pitch_) * qSin(fps_yaw_);
  const float cz = qSin(fps_pitch_);
  QVector3D forward(cx, cy, cz);
  QVector3D right(-cy, cx, 0.f);
  if (!fps_fly_mode_) {
    forward = QVector3D(qCos(fps_yaw_), qSin(fps_yaw_), 0.f);
    if (forward.lengthSquared() > 1e-8f) {
      forward.normalize();
    }
    right = QVector3D(-forward.y(), forward.x(), 0.f);
  }
  if (move_forward_) {
    fps_position_ += forward * speed;
  }
  if (move_backward_) {
    fps_position_ -= forward * speed;
  }
  if (move_right_) {
    fps_position_ += right * speed;
  }
  if (move_left_) {
    fps_position_ -= right * speed;
  }
  if (fps_fly_mode_) {
    if (move_up_) {
      fps_position_.setZ(fps_position_.z() + speed);
    }
    if (move_down_) {
      fps_position_.setZ(fps_position_.z() - speed);
    }
  }
}

bool ViewController::pickGroundPoint(int pixel_x, int pixel_y, int viewport_width,
                                   int viewport_height, QVector3D* hit) const {
  if (hit == nullptr || viewport_width <= 0 || viewport_height <= 0) {
    return false;
  }
  const float aspect =
      static_cast<float>(viewport_width) / static_cast<float>(viewport_height);
  const QMatrix4x4 view = viewMatrix();
  const QMatrix4x4 projection = projectionMatrix(aspect);
  const QMatrix4x4 inverse = (projection * view).inverted();
  const float ndc_x =
      (2.f * static_cast<float>(pixel_x) / static_cast<float>(viewport_width)) -
      1.f;
  const float ndc_y =
      1.f -
      (2.f * static_cast<float>(pixel_y) / static_cast<float>(viewport_height));
  const QVector3D direction = UnprojectPixel(inverse, ndc_x, ndc_y).normalized();
  const QVector3D origin = inverse.map(QVector3D(0.f, 0.f, 0.f));
  return IntersectGroundPlane(origin, direction, hit);
}

void ViewController::appendFocalShape(SceneOverlay* overlay) const {
  if (!focal_shape_visible_ || overlay == nullptr) {
    return;
  }
  if (type_ == ViewControllerType::kFps ||
      type_ == ViewControllerType::kFpsMotion) {
    return;
  }
  const float distance_scale = focal_shape_fixed_size_ ? 1.f : distance_;
  const float radius_x = focal_shape_size_ * distance_scale;
  const float radius_y = focal_shape_size_ * distance_scale;
  const float radius_z = focal_shape_size_ * distance_scale / 5.f;
  overlay->addEllipsoidWireframe(target_, radius_x, radius_y, radius_z,
                                 QColor(255, 255, 0, 128));
}

}  // namespace rendering
}  // namespace autoviz

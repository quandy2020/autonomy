/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/view_controller.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include <Qt>
#include <QtMath>
#include <QVector2D>

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
      QVector3D(near_point.x(), near_point.y(), near_point.z()) /
      std::max(near_point.w(), 1e-6f);
  const QVector3D far_w =
      QVector3D(far_point.x(), far_point.y(), far_point.z()) /
      std::max(far_point.w(), 1e-6f);
  return far_w - near_w;
}

QVector3D UnprojectPoint(const QMatrix4x4& inverse_mvp, float ndc_x, float ndc_y,
                         float ndc_z) {
  const QVector4D clip = inverse_mvp * QVector4D(ndc_x, ndc_y, ndc_z, 1.f);
  return QVector3D(clip.x(), clip.y(), clip.z()) / std::max(clip.w(), 1e-6f);
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

namespace {

// Base-mounted fallback only. Prefer camera_link — eye inside the chassis
// fills the FOV with close mesh (looks like a blurry orange wall).
constexpr float kFpsBaseEyeHeight = 0.18f;
constexpr float kFpsBaseEyeForward = 0.14f;
constexpr float kFpsBasePitchDown = -0.22f;  // rad, look toward the ground plane
constexpr float kFpsNearClip = 0.05f;
constexpr float kFpsFovYDegrees = 70.f;

bool NameEqualsOrEndsWith(const std::string& name, const char* leaf) {
  if (name == leaf) {
    return true;
  }
  const std::string suffix = std::string("/") + leaf;
  return name.size() >= suffix.size() &&
         name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool FrameLooksLikeRobotBase(const std::string& frame) {
  if (frame.empty()) {
    return false;
  }
  return NameEqualsOrEndsWith(frame, "base_link") ||
         NameEqualsOrEndsWith(frame, "base_footprint") ||
         NameEqualsOrEndsWith(frame, "body") ||
         NameEqualsOrEndsWith(frame, "base") ||
         NameEqualsOrEndsWith(frame, "chassis");
}

bool FrameLooksLikeCamera(const std::string& frame) {
  if (frame.empty()) {
    return false;
  }
  return NameEqualsOrEndsWith(frame, "camera_link") ||
         NameEqualsOrEndsWith(frame, "camera_rgb_frame") ||
         NameEqualsOrEndsWith(frame, "camera_rgb_optical_frame") ||
         NameEqualsOrEndsWith(frame, "camera_depth_frame") ||
         NameEqualsOrEndsWith(frame, "camera_depth_optical_frame") ||
         NameEqualsOrEndsWith(frame, "rgb_camera_link") ||
         NameEqualsOrEndsWith(frame, "depth_camera_link");
}

bool FrameLooksLikeLidar(const std::string& frame) {
  return NameEqualsOrEndsWith(frame, "laser_link") ||
         NameEqualsOrEndsWith(frame, "base_scan") ||
         NameEqualsOrEndsWith(frame, "lidar_link") ||
         NameEqualsOrEndsWith(frame, "velodyne");
}

bool FrameLooksLikeFpsMount(const std::string& frame) {
  return FrameLooksLikeCamera(frame) || FrameLooksLikeLidar(frame) ||
         FrameLooksLikeRobotBase(frame);
}

std::string FindFrameByPreference(
    const std::vector<std::string>& names,
    const std::initializer_list<const char*>& preferred) {
  for (const char* pref : preferred) {
    for (const auto& name : names) {
      if (NameEqualsOrEndsWith(name, pref)) {
        return name;
      }
    }
  }
  return {};
}

std::string FindPreferredFpsFrame(const std::vector<std::string>& names) {
  // Camera first (true robot FPS), then lidar, then chassis.
  return FindFrameByPreference(
      names, {"camera_link", "camera_rgb_optical_frame", "camera_rgb_frame",
              "camera_depth_optical_frame", "camera_depth_frame",
              "rgb_camera_link", "laser_link", "base_scan", "lidar_link",
              "base_link", "base_footprint", "body", "base", "chassis"});
}

std::string FindPreferredBaseFrame(const std::vector<std::string>& names) {
  return FindFrameByPreference(
      names, {"base_link", "base_footprint", "body", "base", "chassis"});
}

// Third-person chase defaults (REP-103 +X forward): eye behind on −X.
constexpr float kThirdPersonDistance = 2.5f;
constexpr float kThirdPersonYaw = 3.14159265f;  // π → behind robot
constexpr float kThirdPersonPitch = 0.55f;      // ~31° elevation

}  // namespace

void ViewController::adoptCurrentViewForFps() {
  // Eye / look from the active local matrix (Orbit, Ortho, ThirdPerson, …)
  // expressed in the same frame as fps_position_ (target frame when tracking).
  const QMatrix4x4 inv = localViewMatrix().inverted();
  fps_position_ = inv.map(QVector3D(0.f, 0.f, 0.f));
  QVector3D forward = inv.mapVector(QVector3D(0.f, 0.f, -1.f));
  if (forward.lengthSquared() > 1e-12f) {
    forward.normalize();
    fps_yaw_ = qAtan2(forward.y(), forward.x());
    fps_pitch_ = qAsin(qBound(-1.f, forward.z(), 1.f));
  }
  // Orbit often sits high above the map; drop to a walking-height eye near the
  // focal point so FPS is on the scene, not in the sky.
  const float height_above = fps_position_.z() - target_.z();
  const float horiz =
      QVector2D(fps_position_.x() - target_.x(), fps_position_.y() - target_.y())
          .length();
  if (height_above > 2.f || horiz > 5.f) {
    fps_position_ = QVector3D(target_.x(), target_.y(),
                              target_.z() + kFpsBaseEyeHeight);
    fps_yaw_ = yaw_;
    fps_pitch_ = kFpsBasePitchDown;
  }
}

bool ViewController::isFirstPersonTargetFrame(const std::string& frame) {
  return FrameLooksLikeCamera(frame) || FrameLooksLikeLidar(frame);
}

bool ViewController::placeFpsEyeOnFrame(const std::string& frame) {
  if (frame_manager_ == nullptr || frame.empty()) {
    return false;
  }

  QVector3D position;
  QQuaternion orientation;
  if (!frame_manager_->getTransform(frame, &position, &orientation)) {
    return false;
  }

  // Track the robot frame; eye is expressed in that frame (not fixed/map).
  if (frame != target_frame_) {
    target_frame_ = frame;
    has_target_frame_transform_ = false;
  }
  target_ = QVector3D(0.f, 0.f, 0.f);
  // Clip the robot shell that sits centimetres in front of a body-mounted eye.
  near_clip_distance_ = std::max(near_clip_distance_, kFpsNearClip);

  if (FrameLooksLikeCamera(frame) || FrameLooksLikeLidar(frame)) {
    // Sensor frame: sit at the optical / scan origin looking +X (REP-103).
    fps_position_ = QVector3D(0.f, 0.f, 0.f);
    fps_yaw_ = 0.f;
    fps_pitch_ = 0.f;
  } else if (FrameLooksLikeRobotBase(frame)) {
    // Chassis: nudge forward/up and pitch down so the body does not fill the
    // FOV and the ground plane is not a razor-thin aliased horizon band.
    fps_position_ = QVector3D(kFpsBaseEyeForward, 0.f, kFpsBaseEyeHeight);
    fps_yaw_ = 0.f;
    fps_pitch_ = kFpsBasePitchDown;
  } else {
    fps_position_ = QVector3D(0.f, 0.f, 0.f);
    fps_yaw_ = 0.f;
    fps_pitch_ = 0.f;
  }
  return true;
}

bool ViewController::placeFpsEyeOnRobotFrame() {
  if (frame_manager_ == nullptr) {
    return false;
  }

  // Prefer an already-selected sensor Target Frame (user picked camera_link).
  if (isFirstPersonTargetFrame(target_frame_) &&
      placeFpsEyeOnFrame(target_frame_)) {
    return true;
  }

  // Otherwise prefer camera_link over base_link — the latter puts the eye
  // inside the chassis mesh (blurry close-up of the body).
  std::string frame = FindPreferredFpsFrame(frame_manager_->allFrameNames());
  if (frame.empty() && FrameLooksLikeFpsMount(target_frame_)) {
    frame = target_frame_;
  }
  if (frame.empty()) {
    return false;
  }
  return placeFpsEyeOnFrame(frame);
}

bool ViewController::placeThirdPersonOnRobotFrame() {
  if (frame_manager_ == nullptr) {
    return false;
  }

  std::string frame = FindPreferredBaseFrame(frame_manager_->allFrameNames());
  if (frame.empty() && FrameLooksLikeRobotBase(target_frame_)) {
    frame = target_frame_;
  }
  if (frame.empty()) {
    return false;
  }

  QVector3D position;
  QQuaternion orientation;
  if (!frame_manager_->getTransform(frame, &position, &orientation)) {
    return false;
  }

  if (frame != target_frame_) {
    target_frame_ = frame;
    has_target_frame_transform_ = false;
  }
  target_ = QVector3D(0.f, 0.f, 0.f);
  distance_ = kThirdPersonDistance;
  yaw_ = kThirdPersonYaw;
  pitch_ = kThirdPersonPitch;
  return true;
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
  if (state.target_frame != target_frame_) {
    has_target_frame_transform_ = false;
  }
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
  std::string new_frame;
  if (!trimmed.isEmpty() && trimmed != ViewTargetFrameFixedSentinel()) {
    new_frame = trimmed.toStdString();
  }
  if (new_frame == target_frame_) {
    return;
  }

  // Keep the same fixed-frame focal / FPS eye when the reference frame changes
  // (RViz OrbitViewController::onTargetFrameChanged).
  target_ = reexpressPoint(target_, target_frame_, new_frame);
  fps_position_ = reexpressPoint(fps_position_, target_frame_, new_frame);
  target_frame_ = std::move(new_frame);
  has_target_frame_transform_ = false;
}

bool ViewController::tracksTargetFrame() const { return !target_frame_.empty(); }

QVector3D ViewController::reexpressPoint(const QVector3D& point,
                                         const std::string& from_frame,
                                         const std::string& to_frame) const {
  if (frame_manager_ == nullptr || from_frame == to_frame) {
    return point;
  }

  QVector3D from_pos(0.f, 0.f, 0.f);
  QVector3D to_pos(0.f, 0.f, 0.f);
  QQuaternion from_ori;
  QQuaternion to_ori;
  if (!from_frame.empty() &&
      !frame_manager_->getTransform(from_frame, &from_pos, &from_ori)) {
    return point;
  }
  if (!to_frame.empty() &&
      !frame_manager_->getTransform(to_frame, &to_pos, &to_ori)) {
    return point;
  }

  QVector3D fixed = point;
  if (!from_frame.empty()) {
    fixed = from_ori.rotatedVector(point) + from_pos;
  }
  if (to_frame.empty()) {
    return fixed;
  }
  return to_ori.conjugated().rotatedVector(fixed - to_pos);
}

bool ViewController::resolveTargetFrameToFixed(QMatrix4x4* out) const {
  if (out == nullptr) {
    return false;
  }
  out->setToIdentity();
  if (!tracksTargetFrame() || frame_manager_ == nullptr) {
    return false;
  }

  QVector3D position;
  QQuaternion orientation;
  if (!frame_manager_->getTransform(target_frame_, &position, &orientation)) {
    if (has_target_frame_transform_) {
      *out = last_target_to_fixed_;
      return true;
    }
    return false;
  }

  QMatrix4x4 matrix;
  matrix.translate(position);
  matrix.rotate(orientation);
  last_target_to_fixed_ = matrix;
  has_target_frame_transform_ = true;
  *out = matrix;
  return true;
}

bool ViewController::resolveThirdPersonTargetToFixed(QMatrix4x4* out) const {
  if (out == nullptr) {
    return false;
  }
  out->setToIdentity();
  if (!tracksTargetFrame() || frame_manager_ == nullptr) {
    return false;
  }

  QVector3D position;
  QQuaternion orientation;
  if (!frame_manager_->getTransform(target_frame_, &position, &orientation)) {
    if (has_target_frame_transform_) {
      *out = last_target_to_fixed_;
      return true;
    }
    return false;
  }

  // RViz ThirdPersonFollowerViewController::updateTargetSceneNode — yaw about
  // Z only (REP-103), so pitch/roll of camera_link does not tip the chase cam.
  const QVector3D forward = orientation.rotatedVector(QVector3D(1.f, 0.f, 0.f));
  const float yaw_rad = qAtan2(forward.y(), forward.x());
  QMatrix4x4 matrix;
  matrix.translate(position);
  matrix.rotate(QQuaternion::fromAxisAndAngle(
      QVector3D(0.f, 0.f, 1.f), qRadiansToDegrees(yaw_rad)));
  last_target_to_fixed_ = matrix;
  has_target_frame_transform_ = true;
  *out = matrix;
  return true;
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
  const float fov_y =
      (type_ == ViewControllerType::kFps ||
       type_ == ViewControllerType::kFpsMotion)
          ? kFpsFovYDegrees
          : 45.f;
  projection.perspective(fov_y, aspect_ratio, near_clip_distance_, far_plane);
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

QMatrix4x4 ViewController::topDownOrthoViewMatrix() const {
  // RViz FixedOrientationOrtho / TopDownOrtho: camera sits directly above the
  // focal point on +Z and looks straight down. An XY eye offset (legacy
  // topDownViewMatrix) foreshortens the ground plane so Grid cells stretch.
  const float height = std::max(distance_, 1.f);
  const QVector3D eye(target_.x(), target_.y(), target_.z() + height);
  const float c = qCos(yaw_);
  const float s = qSin(yaw_);
  // yaw = 0 → up = +Y (RViz Angle default).
  const QVector3D up(-s, c, 0.f);
  QMatrix4x4 view;
  view.lookAt(eye, target_, up);
  return view;
}

QMatrix4x4 ViewController::thirdPersonViewMatrix() const {
  // RViz ThirdPersonFollower = XYOrbit relative to the target: yaw/pitch/distance
  // about the focal point. Pitch stays above the ground plane.
  const float pitch = std::max(0.05f, pitch_);
  const float x = distance_ * qCos(yaw_) * qCos(pitch);
  const float y = distance_ * qSin(yaw_) * qCos(pitch);
  const float z = distance_ * qSin(pitch);
  const QVector3D eye = target_ + QVector3D(x, y, z);
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
  QMatrix4x4 target_to_fixed;
  bool have_target = false;
  if (type_ == ViewControllerType::kThirdPersonFollow) {
    // RViz: chase camera turns with target yaw only (ignore pitch/roll).
    have_target = resolveThirdPersonTargetToFixed(&target_to_fixed);
  } else {
    have_target = resolveTargetFrameToFixed(&target_to_fixed);
  }
  if (have_target) {
    // Pose is expressed in the target frame; compose with inv(T) so the
    // camera follows that frame in the fixed/world frame.
    view = view * target_to_fixed.inverted();
  }
  return view;
}

QMatrix4x4 ViewController::localViewMatrix() const {
  switch (type_) {
    case ViewControllerType::kTopDown:
      return topDownViewMatrix();
    case ViewControllerType::kTopDownOrtho:
      return topDownOrthoViewMatrix();
    case ViewControllerType::kXyOrbit:
      return orbitViewMatrix();
    case ViewControllerType::kThirdPersonFollow:
      return thirdPersonViewMatrix();
    case ViewControllerType::kFps:
    case ViewControllerType::kFpsMotion:
      return fpsViewMatrix();
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
  if (type_ == ViewControllerType::kTopDown) {
    return;
  }
  if (type_ == ViewControllerType::kXyOrbit) {
    return;
  }
  if (type_ == ViewControllerType::kThirdPersonFollow) {
    // Stay above the ground plane (XYOrbit-style elevation).
    pitch_ = qBound(0.05f, pitch_ + delta_pitch, 1.4f);
    return;
  }
  pitch_ = qBound(-1.4f, pitch_ + delta_pitch, 1.4f);
}

void ViewController::panBy(float delta_x, float delta_y) {
  const float scale = distance_ * 0.002f;
  if (type_ == ViewControllerType::kFps ||
      type_ == ViewControllerType::kFpsMotion) {
    const float cos_yaw = qCos(fps_yaw_);
    const float sin_yaw = qSin(fps_yaw_);
    const QVector3D right(cos_yaw, sin_yaw, 0.f);
    const QVector3D forward(-sin_yaw, cos_yaw, 0.f);
    fps_position_ -= right * (delta_x * scale);
    fps_position_ += forward * (delta_y * scale);
    return;
  }
  const float cos_yaw = qCos(yaw_);
  const float sin_yaw = qSin(yaw_);
  const QVector3D right(cos_yaw, sin_yaw, 0.f);
  const QVector3D forward(-sin_yaw, cos_yaw, 0.f);
  target_ -= right * (delta_x * scale);
  target_ += forward * (delta_y * scale);
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
  // cameraLocalToWorld() returns a fixed-frame delta (via the full viewMatrix).
  // When tracking, target_ / fps_position_ live in the target frame.
  QVector3D local_delta = delta;
  if (tracksTargetFrame() && frame_manager_ != nullptr) {
    QVector3D position;
    QQuaternion orientation;
    if (frame_manager_->getTransform(target_frame_, &position, &orientation)) {
      local_delta = orientation.conjugated().rotatedVector(delta);
    }
  }
  target_ += local_delta;
  if (type_ == ViewControllerType::kFps) {
    fps_position_ += local_delta;
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
  if (type_ == ViewControllerType::kFps ||
      type_ == ViewControllerType::kFpsMotion) {
    // Dolly along look direction — fpsViewMatrix does not use distance_.
    const float cx = qCos(fps_pitch_) * qCos(fps_yaw_);
    const float cy = qCos(fps_pitch_) * qSin(fps_yaw_);
    const float cz = qSin(fps_pitch_);
    fps_position_ += QVector3D(cx, cy, cz) * amount;
    return;
  }
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
      last_mouse_x_ = event.x;
      last_mouse_y_ = event.y;
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
        const int diff_x = event.x - last_mouse_x_;
        const int diff_y = event.y - last_mouse_y_;
        last_mouse_x_ = event.x;
        last_mouse_y_ = event.y;
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
  const float cx = qCos(fps_pitch_) * qCos(fps_yaw_);
  const float cy = qCos(fps_pitch_) * qSin(fps_yaw_);
  const float cz = qSin(fps_pitch_);
  QVector3D forward(cx, cy, cz);
  QVector3D right(-cy, cx, 0.f);
  if (!fps_fly_mode_) {
    // Walk: horizontal move; look pitch only affects mouse look.
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

bool ViewController::pickRay(int pixel_x, int pixel_y, int viewport_width,
                             int viewport_height, QVector3D* origin,
                             QVector3D* direction) const {
  if (origin == nullptr || direction == nullptr || viewport_width <= 0 ||
      viewport_height <= 0) {
    return false;
  }
  const float aspect =
      static_cast<float>(viewport_width) / static_cast<float>(viewport_height);
  const QMatrix4x4 inverse =
      (projectionMatrix(aspect) * viewMatrix()).inverted();
  const float ndc_x =
      (2.f * static_cast<float>(pixel_x) / static_cast<float>(viewport_width)) -
      1.f;
  const float ndc_y =
      1.f -
      (2.f * static_cast<float>(pixel_y) / static_cast<float>(viewport_height));
  *origin = UnprojectPoint(inverse, ndc_x, ndc_y, -1.f);
  const QVector3D dir = UnprojectPixel(inverse, ndc_x, ndc_y);
  if (dir.lengthSquared() < 1e-12f) {
    return false;
  }
  *direction = dir.normalized();
  return true;
}

bool ViewController::pickGroundPoint(int pixel_x, int pixel_y, int viewport_width,
                                   int viewport_height, QVector3D* hit) const {
  QVector3D origin;
  QVector3D direction;
  if (!pickRay(pixel_x, pixel_y, viewport_width, viewport_height, &origin,
               &direction)) {
    return false;
  }
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
  QVector3D focal_fixed = target_;
  QMatrix4x4 target_to_fixed;
  if (resolveTargetFrameToFixed(&target_to_fixed)) {
    focal_fixed = target_to_fixed.map(target_);
  }
  const float distance_scale = focal_shape_fixed_size_ ? 1.f : distance_;
  const float radius_x = focal_shape_size_ * distance_scale;
  const float radius_y = focal_shape_size_ * distance_scale;
  const float radius_z = focal_shape_size_ * distance_scale / 5.f;
  overlay->addEllipsoidWireframe(focal_fixed, radius_x, radius_y, radius_z,
                                 QColor(255, 255, 0, 128));
}

}  // namespace rendering
}  // namespace autoviz

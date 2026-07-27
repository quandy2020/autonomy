/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>

#include <Qt>
#include <QMatrix4x4>
#include <QVector3D>

namespace autoviz {
namespace common {
class FrameManager;
}

namespace rendering {

class SceneOverlay;

inline QString ViewTargetFrameFixedSentinel() {
  return QStringLiteral("<Fixed Frame>");
}

enum class ViewControllerType {
  kOrbit,
  kXyOrbit,
  kTopDown,
  kTopDownOrtho,
  kFps,
  kFpsMotion,
  kThirdPersonFollow
};

struct ViewState {
  ViewControllerType type = ViewControllerType::kOrbit;
  float near_clip_distance = 0.01f;
  bool invert_z_axis = false;
  float yaw = 0.785398f;
  float pitch = 0.785398f;
  float distance = 10.f;
  QVector3D target{0.f, 0.f, 0.f};
  float focal_shape_size = 0.05f;
  bool focal_shape_fixed_size = true;
  QVector3D fps_position{0.f, 0.f, 2.f};
  float fps_yaw = 3.14f;
  float fps_pitch = 0.f;
  /** Empty means track the global fixed frame (shown as "<Fixed Frame>"). */
  std::string target_frame;
};

/** RViz-style viewport mouse event forwarded from the render widget. */
struct ViewportMouseEvent {
  enum class Action { kPress, kRelease, kMove, kWheel };

  Action action = Action::kMove;
  Qt::MouseButtons buttons = Qt::NoButton;
  Qt::KeyboardModifiers modifiers = Qt::NoModifier;
  int x = 0;
  int y = 0;
  int wheel_delta = 0;
  int viewport_width = 1;
  int viewport_height = 1;
};

/** Camera controller (Orbit / XYOrbit / TopDown / TopDownOrtho / FPS / ThirdPersonFollow). */
class ViewController {
 public:
  ViewControllerType type() const { return type_; }
  void setType(ViewControllerType type);
  void setTypeByName(const QString& name);
  QString typeName() const;

  ViewState state() const;
  void setState(const ViewState& state);
  void reset();

  void setFrameManager(common::FrameManager* frame_manager) {
    frame_manager_ = frame_manager;
  }

  QString targetFrameDisplay() const;
  void setTargetFrame(const QString& frame);
  bool tracksTargetFrame() const;

  void setFocalShapeVisible(bool visible) { focal_shape_visible_ = visible; }
  bool focalShapeVisible() const { return focal_shape_visible_; }
  void appendFocalShape(rendering::SceneOverlay* overlay) const;

  float nearClipDistance() const { return near_clip_distance_; }
  void setNearClipDistance(float value);

  bool invertZAxis() const { return invert_z_axis_; }
  void setInvertZAxis(bool value);

  float focalShapeSize() const { return focal_shape_size_; }
  void setFocalShapeSize(float value);

  bool focalShapeFixedSize() const { return focal_shape_fixed_size_; }
  void setFocalShapeFixedSize(bool value);

  QMatrix4x4 viewMatrix() const;
  QMatrix4x4 projectionMatrix(float aspect_ratio) const;

  void orbitBy(float delta_yaw, float delta_pitch);
  void panBy(float delta_x, float delta_y);
  void zoomBy(float delta_distance);

  /** OrbitViewController-compatible mouse handling (Move Camera tool forwards here). */
  bool handleMouseEvent(const ViewportMouseEvent& event);
  bool isViewDragging() const { return dragging_; }

  void setTarget(const QVector3D& target);
  const QVector3D& target() const { return target_; }

  void setFpsKey(int key, bool pressed);
  /** F (walk/fly toggle) and R (reset) for FPS / FPSMotion. Returns true if handled. */
  bool handleKeyEvent(int key, bool pressed);
  void tick(float delta_seconds);

  bool fpsFlyMode() const { return fps_fly_mode_; }

  bool pickGroundPoint(int pixel_x, int pixel_y, int viewport_width,
                       int viewport_height, QVector3D* hit) const;

 private:
  enum class ViewDragMode { kNone, kOrbit, kPanXY, kZoom, kPanZ };

  QMatrix4x4 localViewMatrix() const;
  QMatrix4x4 orbitViewMatrix() const;
  QMatrix4x4 topDownViewMatrix() const;
  QMatrix4x4 thirdPersonViewMatrix() const;
  QMatrix4x4 fpsViewMatrix() const;
  QMatrix4x4 targetFrameToFixedMatrix() const;

  void rotateCamera(float diff_x, float diff_y);
  void panFocalPointXY(float diff_x, float diff_y, float aspect_ratio,
                       int viewport_width, int viewport_height);
  void panFocalPointZ(float amount);
  void zoomCamera(float amount);
  void applyFocalDelta(const QVector3D& delta);
  QVector3D cameraLocalToWorld(float local_x, float local_y, float local_z) const;
  ViewDragMode dragModeForPress(Qt::MouseButtons buttons,
                                Qt::KeyboardModifiers modifiers) const;

  ViewControllerType type_ = ViewControllerType::kOrbit;
  float near_clip_distance_ = 0.01f;
  bool invert_z_axis_ = false;
  float yaw_ = 0.785398f;
  float pitch_ = 0.785398f;
  float distance_ = 10.f;
  QVector3D target_{0.f, 0.f, 0.f};
  float focal_shape_size_ = 0.05f;
  bool focal_shape_fixed_size_ = true;

  QVector3D fps_position_{0.f, 0.f, 2.f};
  float fps_yaw_ = 3.14f;
  float fps_pitch_ = 0.f;
  bool fps_fly_mode_ = false;
  bool move_forward_ = false;
  bool move_backward_ = false;
  bool move_left_ = false;
  bool move_right_ = false;
  bool move_up_ = false;
  bool move_down_ = false;
  bool focal_shape_visible_ = false;
  std::string target_frame_;
  common::FrameManager* frame_manager_ = nullptr;
  float xy_orbit_pitch_ = 0.65f;
  ViewDragMode drag_mode_ = ViewDragMode::kNone;
  bool dragging_ = false;
  int last_mouse_x_ = 0;
  int last_mouse_y_ = 0;
};

}  // namespace rendering
}  // namespace autoviz

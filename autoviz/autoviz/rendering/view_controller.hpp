/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file view_controller.hpp
 * @brief RViz-style camera controllers (Orbit, Ortho, FPS, ThirdPersonFollow).
 *
 * Owns yaw/pitch/distance or FPS eye state, produces view/projection matrices
 * for OpenGL and Ogre viewports, and handles mouse/keyboard interaction used by
 * the Move Camera tool and Views panel.
 *
 * @see ViewsPanel
 * @see ViewportMouseEvent
 * @see SceneOverlay
 * @see common::FrameManager
 */

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

/**
 * @brief Sentinel string shown when the camera tracks the global fixed frame.
 *
 * Stored UI value for an empty @ref ViewState::target_frame.
 *
 * @return Literal @c "<Fixed Frame>".
 * @see ViewController::targetFrameDisplay()
 */
inline QString ViewTargetFrameFixedSentinel() {
  return QStringLiteral("<Fixed Frame>");
}

/**
 * @enum ViewControllerType
 * @brief Built-in camera interaction modes (RViz2-aligned + Autoviz aliases).
 *
 * @c kTopDown / @c kFpsMotion are legacy aliases mapped by name helpers to
 * @c kTopDownOrtho / @c kFps where appropriate.
 */
enum class ViewControllerType {
  kOrbit,              /**< Free orbit about a focal point. */
  kXyOrbit,            /**< Orbit with constrained pitch (XY preference). */
  kTopDown,            /**< Legacy alias for top-down perspective. */
  kTopDownOrtho,       /**< Orthographic top-down (RViz TopDownOrtho). */
  kFps,                /**< First-person look + WASD. */
  kFpsMotion,          /**< Legacy FPS motion alias. */
  kThirdPersonFollow   /**< Follow a target frame behind the focal point. */
};

/**
 * @struct ViewState
 * @brief Serializable snapshot of @ref ViewController parameters.
 *
 * Persisted in session config / saved Views bookmarks. Empty
 * @ref target_frame means track the global fixed frame (UI shows
 * @ref ViewTargetFrameFixedSentinel()).
 *
 * @see ViewController::state()
 * @see ViewController::setState()
 * @see common::SavedViewConfig
 */
struct ViewState {
  ViewControllerType type = ViewControllerType::kOrbit; /**< Controller mode. */
  float near_clip_distance = 0.01f; /**< Near clip plane (meters). */
  bool invert_z_axis = false;       /**< Invert Z when building matrices. */
  float yaw = 0.785398f;            /**< Orbit/ortho yaw (rad). */
  float pitch = 0.785398f;          /**< Orbit pitch (rad). */
  float distance = 10.f;            /**< Orbit distance or ortho scale. */
  QVector3D target{0.f, 0.f, 0.f};  /**< Focal / look-at point. */
  float focal_shape_size = 0.05f;   /**< Focal marker size. */
  bool focal_shape_fixed_size = true; /**< Marker ignores distance scaling. */
  QVector3D fps_position{0.f, 0.f, 2.f}; /**< FPS eye position. */
  float fps_yaw = 3.14f;            /**< FPS yaw (rad). */
  float fps_pitch = 0.f;            /**< FPS pitch (rad). */
  /** Empty means track the global fixed frame (shown as "<Fixed Frame>"). */
  std::string target_frame;
};

/**
 * @struct ViewportMouseEvent
 * @brief RViz-style viewport mouse event forwarded from the render widget.
 *
 * Built by helpers in @ref viewport_mouse.hpp; consumed by
 * @ref ViewController::handleMouseEvent().
 */
struct ViewportMouseEvent {
  /**
   * @enum Action
   * @brief Mouse / wheel phase.
   */
  enum class Action {
    kPress,   /**< Button down. */
    kRelease, /**< Button up. */
    kMove,    /**< Pointer move (with or without buttons). */
    kWheel    /**< Scroll wheel (@ref wheel_delta). */
  };

  Action action = Action::kMove;                 /**< Event phase. */
  Qt::MouseButtons buttons = Qt::NoButton;       /**< Buttons held. */
  Qt::KeyboardModifiers modifiers = Qt::NoModifier; /**< Modifier keys. */
  int x = 0;                 /**< Cursor X in widget pixels. */
  int y = 0;                 /**< Cursor Y in widget pixels. */
  int wheel_delta = 0;       /**< Vertical wheel delta (Qt angleDelta.y). */
  int viewport_width = 1;    /**< Viewport width for aspect / pan. */
  int viewport_height = 1;   /**< Viewport height. */
};

/**
 * @class ViewController
 * @brief Camera controller (Orbit / XYOrbit / TopDown / TopDownOrtho / FPS /
 *        ThirdPersonFollow).
 *
 * ## Responsibilities
 *
 * - Maintain interaction state (yaw, pitch, distance, FPS pose, target frame).
 * - Emit @ref viewMatrix() / @ref projectionMatrix() for rendering.
 * - Handle Move Camera mouse and FPS keys; optional focal-shape overlay.
 *
 * ## Target frame
 *
 * When @ref tracksTargetFrame() is true, view matrices are composed with
 * @ref common::FrameManager transforms so the camera follows a TF frame.
 *
 * @note Does not own @ref common::FrameManager or @ref SceneOverlay.
 *
 * @see ViewsPanel
 * @see RenderWindow
 * @see OgreRenderWindow
 */
class ViewController {
 public:
  /**
   * @brief Returns the active controller type.
   * @return Current @ref ViewControllerType.
   */
  ViewControllerType type() const { return type_; }

  /**
   * @brief Switches type and resets drag state as needed.
   * @param type New controller mode.
   */
  void setType(ViewControllerType type);

  /**
   * @brief Sets type from an RViz / Autoviz type name string.
   *
   * Accepts names such as @c Orbit, @c XYOrbit, @c TopDownOrtho, @c FPS,
   * and legacy aliases (@c TopDown, @c FPSMotion).
   *
   * @param name Type id (case-sensitive display/session string).
   */
  void setTypeByName(const QString& name);

  /**
   * @brief Returns the canonical type name for the current mode.
   * @return QString suitable for combo boxes / session config.
   */
  QString typeName() const;

  /**
   * @brief Snapshots all parameters into a @ref ViewState.
   * @return Copy of current state.
   */
  ViewState state() const;

  /**
   * @brief Restores parameters from a @ref ViewState (e.g. saved view).
   * @param state Snapshot to apply.
   */
  void setState(const ViewState& state);

  /**
   * @brief Resets to defaults for the current type (Views panel Zero).
   */
  void reset();

  /**
   * @brief Attaches the TF frame manager for target-frame tracking.
   * @param frame_manager Non-owning; may be @c nullptr.
   */
  void setFrameManager(common::FrameManager* frame_manager) {
    frame_manager_ = frame_manager;
  }

  /**
   * @brief Display string for Target Frame (or @ref ViewTargetFrameFixedSentinel()).
   * @return Frame name or fixed-frame sentinel.
   */
  QString targetFrameDisplay() const;

  /**
   * @brief Sets the TF target frame; empty / sentinel clears to fixed frame.
   * @param frame Frame name or @ref ViewTargetFrameFixedSentinel().
   */
  void setTargetFrame(const QString& frame);

  /**
   * @brief Whether a non-empty target frame is being tracked.
   * @return @c true if @c target_frame_ is non-empty.
   */
  bool tracksTargetFrame() const;

  /**
   * @brief Enables drawing of the focal-point marker into an overlay.
   * @param visible When @c true, @ref appendFocalShape() emits geometry.
   */
  void setFocalShapeVisible(bool visible) { focal_shape_visible_ = visible; }

  /**
   * @brief Whether the focal shape should be drawn.
   * @return Current visibility flag.
   */
  bool focalShapeVisible() const { return focal_shape_visible_; }

  /**
   * @brief Appends focal-shape geometry to @p overlay when visible.
   * @param overlay Destination overlay; no-op if null or not visible.
   */
  void appendFocalShape(rendering::SceneOverlay* overlay) const;

  /** @brief Near clip distance in meters. */
  float nearClipDistance() const { return near_clip_distance_; }

  /**
   * @brief Sets the near clip plane distance.
   * @param value Distance in meters (clamped by implementation).
   */
  void setNearClipDistance(float value);

  /** @brief Whether the Z axis is inverted in matrices. */
  bool invertZAxis() const { return invert_z_axis_; }

  /**
   * @brief Enables or disables Z-axis inversion.
   * @param value New invert flag.
   */
  void setInvertZAxis(bool value);

  /** @brief Focal marker size. */
  float focalShapeSize() const { return focal_shape_size_; }

  /**
   * @brief Sets focal marker size.
   * @param value Size in world units (or screen-scaled when fixed).
   */
  void setFocalShapeSize(float value);

  /** @brief Whether focal size ignores camera distance. */
  bool focalShapeFixedSize() const { return focal_shape_fixed_size_; }

  /**
   * @brief Sets fixed vs distance-scaled focal marker size.
   * @param value @c true for fixed size.
   */
  void setFocalShapeFixedSize(bool value);

  /**
   * @brief Builds the camera view matrix for the current type / target frame.
   * @return 4×4 view matrix (world → camera).
   */
  QMatrix4x4 viewMatrix() const;

  /**
   * @brief Builds the projection matrix (perspective or ortho).
   *
   * @param aspect_ratio Viewport width / height.
   * @return 4×4 projection matrix.
   */
  QMatrix4x4 projectionMatrix(float aspect_ratio) const;

  /**
   * @brief Applies an Orbit yaw/pitch delta (radians).
   * @param delta_yaw Change in yaw.
   * @param delta_pitch Change in pitch.
   */
  void orbitBy(float delta_yaw, float delta_pitch);

  /**
   * @brief Pans the focal point in camera-local X/Y.
   * @param delta_x Horizontal pan amount.
   * @param delta_y Vertical pan amount.
   */
  void panBy(float delta_x, float delta_y);

  /**
   * @brief Zooms by changing orbit distance (or ortho scale).
   * @param delta_distance Signed distance change.
   */
  void zoomBy(float delta_distance);

  /**
   * @brief OrbitViewController-compatible mouse handling.
   *
   * Move Camera tool and default viewport interaction forward events here.
   *
   * @param event Normalized viewport mouse event.
   * @return @c true if the event changed the view (caller should redraw).
   */
  bool handleMouseEvent(const ViewportMouseEvent& event);

  /**
   * @brief Whether a drag interaction is in progress.
   * @return @c true between press and matching release for a drag mode.
   */
  bool isViewDragging() const { return dragging_; }

  /**
   * @brief Sets the orbit / look-at focal point.
   * @param target World-space target.
   */
  void setTarget(const QVector3D& target);

  /**
   * @brief Returns the current focal / look-at point.
   * @return Const reference to @c target_.
   */
  const QVector3D& target() const { return target_; }

  /**
   * @brief Sets an FPS movement key state (WASD / up / down).
   * @param key Qt key code.
   * @param pressed @c true on key press, @c false on release.
   */
  void setFpsKey(int key, bool pressed);

  /**
   * @brief Handles F (walk/fly toggle) and R (reset) for FPS modes.
   *
   * @param key Qt key code.
   * @param pressed Press or release.
   * @return @c true if the key was handled.
   */
  bool handleKeyEvent(int key, bool pressed);

  /**
   * @brief Integrates FPS motion for @p delta_seconds.
   * @param delta_seconds Elapsed time since last tick.
   */
  void tick(float delta_seconds);

  /**
   * @brief Whether FPS fly mode (vs walk) is active.
   * @return Current fly-mode flag.
   */
  bool fpsFlyMode() const { return fps_fly_mode_; }

  /**
   * @brief Ray–ground intersection for tools (Z=0 plane in fixed frame).
   *
   * @param pixel_x Cursor X.
   * @param pixel_y Cursor Y.
   * @param viewport_width Viewport width.
   * @param viewport_height Viewport height.
   * @param[out] hit Filled with world hit on success.
   * @return @c true if the ray intersects the ground plane.
   *
   * @see ViewportProjectionFinder::projectOnGroundPlane()
   */
  bool pickGroundPoint(int pixel_x, int pixel_y, int viewport_width,
                       int viewport_height, QVector3D* hit) const;

 private:
  /**
   * @enum ViewDragMode
   * @brief Active mouse-drag interaction.
   */
  enum class ViewDragMode {
    kNone,   /**< Not dragging. */
    kOrbit,  /**< Rotate yaw/pitch. */
    kPanXY,  /**< Pan in camera X/Y. */
    kZoom,   /**< Change distance. */
    kPanZ    /**< Pan along camera Z / world Z. */
  };

  QMatrix4x4 localViewMatrix() const;
  QMatrix4x4 orbitViewMatrix() const;
  QMatrix4x4 topDownViewMatrix() const;
  QMatrix4x4 topDownOrthoViewMatrix() const;
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

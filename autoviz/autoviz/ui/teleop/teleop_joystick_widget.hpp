/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file teleop_joystick_widget.hpp
 * @brief Circular on-screen joystick with normalized output in [-1, 1].
 *
 * Used by @ref TeleopControlWidget for Move / Turn / Arcade sticks. Supports
 * omni (X+Y) and yaw-only (X) axis modes, plus programmatic knob updates for
 * keyboard mirroring.
 *
 * @see TeleopControlWidget
 * @see TeleopStickAxes
 */

#pragma once

#include <QWidget>

namespace autoviz {
namespace teleop {

/**
 * @enum TeleopStickAxes
 * @brief Which cardinal axes are interactive / labeled on the pad.
 */
enum class TeleopStickAxes {
  kOmni = 0,   /**< F/B/L/R — Move / Drive (both axes). */
  kYaw = 1,    /**< L/R only — Turn (X axis; Y clamped to 0). */
};

/**
 * @class TeleopJoystickWidget
 * @brief Circular joystick widget emitting normalized @c (x, y) in [-1, 1].
 *
 * ## Interaction
 *
 * - Mouse drag updates the knob and emits @ref valueChanged().
 * - Mouse release emits @ref released() (and typically centers when the
 *   parent requests @ref reset()).
 * - @ref setNormalizedValue() updates the knob without requiring a drag
 *   (keyboard / external control).
 *
 * Widget coordinates: +Y is screen-down; callers usually negate Y for
 * "forward" in robot frame.
 *
 * @note Preferred size is square; @ref sizeHint() / @ref minimumSizeHint()
 *       return a fixed-ish pad size.
 */
class TeleopJoystickWidget : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs a labeled joystick pad.
   *
   * @param label Caption drawn near the pad (e.g. @c "Move", @c "Turn").
   * @param parent Qt parent widget.
   */
  explicit TeleopJoystickWidget(const QString& label, QWidget* parent = nullptr);

  /**
   * @brief Preferred size for layout (roughly square pad).
   *
   * @return Suggested @c QSize.
   */
  QSize sizeHint() const override;

  /**
   * @brief Minimum size so the knob and labels remain usable.
   *
   * @return Minimum @c QSize.
   */
  QSize minimumSizeHint() const override;

  /**
   * @brief Current normalized knob position.
   *
   * @return Point with @c x,@c y each in approximately [-1, 1].
   */
  QPointF normalizedValue() const;

  /**
   * @brief Centers the knob and clears drag state (emits release semantics
   *        as implemented).
   *
   * @see resetVisual()
   */
  void reset();

  /**
   * @brief Centers the knob visually without treating it as a user release.
   *
   * Used when switching stick modes so leftover drag state is cleared quietly.
   */
  void resetVisual();

  /**
   * @brief Updates the knob without mouse drag (keyboard / external control).
   *
   * @param value Desired normalized position (clamped to axes).
   * @param emit_signal When @c true, emits @ref valueChanged().
   */
  void setNormalizedValue(const QPointF& value, bool emit_signal = true);

  /**
   * @brief Replaces the caption drawn on the pad.
   *
   * @param label New label string.
   */
  void setLabel(const QString& label);

  /**
   * @brief Restricts interactive axes (omni vs yaw-only).
   *
   * @param axes Axis mode.
   * @see axes()
   */
  void setAxes(TeleopStickAxes axes);

  /**
   * @brief Returns the current axis restriction.
   *
   * @return @ref TeleopStickAxes value.
   */
  TeleopStickAxes axes() const { return axes_; }

 signals:
  /**
   * @brief Emitted when the normalized knob position changes.
   *
   * @param x Horizontal value in [-1, 1] (right positive).
   * @param y Vertical value in [-1, 1] (screen-down positive).
   */
  void valueChanged(double x, double y);

  /**
   * @brief Emitted when the user releases the mouse after dragging.
   */
  void released();

 protected:
  /**
   * @brief Paints the outer ring, crosshair labels, and knob.
   *
   * @param event Paint event.
   */
  void paintEvent(QPaintEvent* event) override;

  /**
   * @brief Recomputes geometry caches after resize.
   *
   * @param event Resize event.
   */
  void resizeEvent(QResizeEvent* event) override;

  /**
   * @brief Starts a drag if the press is inside the pad.
   *
   * @param event Mouse press event.
   */
  void mousePressEvent(QMouseEvent* event) override;

  /**
   * @brief Updates knob position while dragging.
   *
   * @param event Mouse move event.
   */
  void mouseMoveEvent(QMouseEvent* event) override;

  /**
   * @brief Ends the drag and emits @ref released().
   *
   * @param event Mouse release event.
   */
  void mouseReleaseEvent(QMouseEvent* event) override;

 private:
  /**
   * @brief Maps a local widget position to a clamped normalized knob value.
   *
   * @param local_pos Position in widget coordinates.
   */
  void updateFromPosition(const QPointF& local_pos);

  /**
   * @brief Clamps @p vector according to @c axes_ (e.g. zero Y in yaw mode).
   *
   * @param vector Candidate normalized vector.
   * @return Axis-clamped vector inside the unit circle.
   */
  QPointF clampToAxes(const QPointF& vector) const;

  /**
   * @brief Bounding rect of the outer circle.
   *
   * @return Outer ring rectangle in widget coordinates.
   */
  QRectF outerRect() const;

  /**
   * @brief Center point of the pad.
   *
   * @return Pad center in widget coordinates.
   */
  QPointF center() const;

  /**
   * @brief Outer radius used for clamping drag distance.
   *
   * @return Radius in pixels.
   */
  double radius() const;

  QString label_;                              /**< Caption drawn on the pad. */
  TeleopStickAxes axes_ = TeleopStickAxes::kOmni; /**< Interactive axes. */
  QPointF knob_{0.0, 0.0};                     /**< Normalized knob position. */
  bool dragging_ = false;                      /**< @c true while mouse drag active. */
};

}  // namespace teleop
}  // namespace autoviz

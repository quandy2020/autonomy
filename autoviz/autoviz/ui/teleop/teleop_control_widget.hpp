/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file teleop_control_widget.hpp
 * @brief On-screen teleop controls — dual / arcade sticks, speeds, smart teleop.
 *
 * Emits normalized linear / angular commands consumed by @ref TeleopPanel,
 * which scales them by max speeds and publishes Twist (or smart-teleop goals).
 * Also supports WASD / arrow keyboard driving when focused.
 *
 * @see TeleopJoystickWidget
 * @see TeleopPanel
 * @see TeleopStickMode
 */

#pragma once

#include <QWidget>

#include "autoviz/ui/teleop/teleop_types.hpp"

class QAbstractButton;
class QButtonGroup;
class QCheckBox;
class QDoubleSpinBox;
class QEvent;
class QFocusEvent;
class QFrame;
class QKeyEvent;
class QLabel;

namespace autoviz {
namespace teleop {

class TeleopJoystickWidget;

/**
 * @class TeleopControlWidget
 * @brief Interactive teleop pad: stick mode, max speeds, smart-teleop toggle.
 *
 * ## Layout
 *
 * @code
 * ┌─ [Dual] [Arcade]   max linear / angular ─ smart teleop ─┐
 * │  Dual: [Move stick] [Turn stick]                         │
 * │  Arcade: [single stick]                                  │
 * │  hint / smart-teleop status                              │
 * └──────────────────────────────────────────────────────────┘
 * @endcode
 *
 * ## Signals
 *
 * - @ref linearChanged() — Move stick / Arcade Y (and Dual X strafe).
 * - @ref angularChanged() — Turn stick X / Arcade X.
 * - @ref linearReleased() / @ref angularReleased() — stick mouse-up.
 * - @ref stopClicked() — explicit stop (if wired).
 * - Mode / speed / smart-teleop changes for panel config sync.
 *
 * @note Does not publish on the wire; @ref TeleopPanel owns writers / timers.
 */
class TeleopControlWidget : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Builds dual / arcade sticks, speed spins, and mode toggles.
   *
   * @param parent Qt parent (typically @ref TeleopPanel content).
   */
  explicit TeleopControlWidget(QWidget* parent = nullptr);

  /**
   * @brief Centers all joysticks and clears keyboard driving state.
   *
   * Called when publishing stops or the panel loses active drive state.
   */
  void resetJoysticks();

  /**
   * @brief Switches between Dual and Arcade stick layouts.
   *
   * @param mode @ref TeleopStickMode::kDual or @ref TeleopStickMode::kArcade.
   * @see stickMode()
   * @see stickModeChanged()
   */
  void setStickMode(TeleopStickMode mode);

  /**
   * @brief Returns the current stick layout mode.
   *
   * @return @ref TeleopStickMode value.
   */
  TeleopStickMode stickMode() const { return stick_mode_; }

  /**
   * @brief Sets max linear and angular speeds shown in the spins.
   *
   * @param max_linear Max |linear| in m/s at full stick deflection.
   * @param max_angular Max |angular.z| in rad/s at full stick deflection.
   * @see maxLinearSpeed()
   * @see maxAngularSpeed()
   */
  void setMaxSpeeds(double max_linear, double max_angular);

  /**
   * @brief Current max linear speed (m/s).
   *
   * @return Value of @c max_linear_speed_.
   */
  double maxLinearSpeed() const { return max_linear_speed_; }

  /**
   * @brief Current max angular speed (rad/s).
   *
   * @return Value of @c max_angular_speed_.
   */
  double maxAngularSpeed() const { return max_angular_speed_; }

  /**
   * @brief Enables or clears the smart-teleop checkbox.
   *
   * @param enabled When @c true, panel should route via teleop goals.
   * @see smartTeleopEnabled()
   * @see smartTeleopChanged()
   */
  void setSmartTeleopEnabled(bool enabled);

  /**
   * @brief Whether smart teleop is checked.
   *
   * @return @c true if smart teleop is enabled in the UI.
   */
  bool smartTeleopEnabled() const { return smart_teleop_enabled_; }

  /**
   * @brief Enables/disables the smart-teleop checkbox (feature availability).
   *
   * @param available When @c false, checkbox is disabled / hidden per UI rules.
   */
  void setSmartTeleopAvailable(bool available);

  /**
   * @brief Updates the smart-teleop status / hint text under the sticks.
   *
   * @param status Human-readable connection / session status.
   */
  void setSmartTeleopStatusText(const QString& status);

 signals:
  /**
   * @brief Dual Move stick / Arcade stick Y: @p x = strafe (dual only),
   *        @p y = forward.
   *
   * Values are normalized stick units in roughly [-1, 1]; the panel scales
   * by max speeds.
   *
   * @param x Strafe (Dual) or unused (Arcade).
   * @param y Forward / back.
   */
  void linearChanged(double x, double y);

  /**
   * @brief Dual Turn stick X / Arcade stick X: yaw command.
   *
   * @param turn Normalized turn in [-1, 1].
   */
  void angularChanged(double turn);

  /** @brief Move / Arcade stick released (mouse up). */
  void linearReleased();

  /** @brief Turn stick released (mouse up). */
  void angularReleased();

  /** @brief Explicit stop control activated. */
  void stopClicked();

  /**
   * @brief Stick mode toggle changed.
   *
   * @param mode New @ref TeleopStickMode.
   */
  void stickModeChanged(TeleopStickMode mode);

  /**
   * @brief Max speed spins changed.
   *
   * @param max_linear New max linear (m/s).
   * @param max_angular New max angular (rad/s).
   */
  void maxSpeedsChanged(double max_linear, double max_angular);

  /**
   * @brief Smart-teleop checkbox toggled.
   *
   * @param enabled New checked state.
   */
  void smartTeleopChanged(bool enabled);

 protected:
  /**
   * @brief Starts keyboard driving (WASD / arrows) when focused.
   *
   * @param event Key press event.
   */
  void keyPressEvent(QKeyEvent* event) override;

  /**
   * @brief Clears keyboard driving bits and may emit release.
   *
   * @param event Key release event.
   */
  void keyReleaseEvent(QKeyEvent* event) override;

  /**
   * @brief Ensures the widget can receive key events for driving.
   *
   * @param event Focus event.
   */
  void focusInEvent(QFocusEvent* event) override;

  /**
   * @brief Filters child focus / key events for keyboard teleop.
   *
   * @param watched Object being filtered.
   * @param event Event delivered to @p watched.
   * @return @c true if the event was handled.
   */
  bool eventFilter(QObject* watched, QEvent* event) override;

 private:
  /** @brief Shows Dual or Arcade frame and resets inactive sticks. */
  void applyModeUi();

  /**
   * @brief Maps Arcade stick @p x,@p y into linear + angular signals.
   *
   * @param x Stick X (turn).
   * @param y Stick Y (forward).
   * @param released When @c true, emit release instead of value changes.
   */
  void emitArcadeFromStick(double x, double y, bool released);

  /** @brief Recomputes Arcade outputs from current keyboard bit flags. */
  void updateArcadeFromKeyboard();

  /** @brief Clears @c key_* flags and @c keyboard_driving_. */
  void clearKeyboardState();

  /** @brief Reads speed spins into members and emits @ref maxSpeedsChanged(). */
  void emitMaxSpeedsFromUi();

  /** @brief Refreshes the hint label from smart-teleop status text. */
  void updateSmartTeleopHint();

  TeleopStickMode stick_mode_ = TeleopStickMode::kDual; /**< Active stick layout. */
  bool smart_teleop_enabled_ = false;   /**< Smart-teleop checkbox state. */
  QString smart_teleop_status_text_;    /**< Status line under the sticks. */
  double max_linear_speed_ = 0.5;       /**< Max |linear| m/s. */
  double max_angular_speed_ = 0.5;      /**< Max |angular.z| rad/s. */
  QButtonGroup* mode_group_ = nullptr; /**< Dual vs Arcade exclusive group. */
  QAbstractButton* dual_mode_button_ = nullptr;   /**< Dual mode toggle. */
  QAbstractButton* arcade_mode_button_ = nullptr; /**< Arcade mode toggle. */
  QDoubleSpinBox* max_linear_spin_ = nullptr;     /**< Max linear editor. */
  QDoubleSpinBox* max_angular_spin_ = nullptr;    /**< Max angular editor. */
  QFrame* dual_frame_ = nullptr;        /**< Container for Move + Turn sticks. */
  QFrame* arcade_frame_ = nullptr;      /**< Container for Arcade stick. */
  TeleopJoystickWidget* move_joystick_ = nullptr;   /**< Dual Move pad. */
  TeleopJoystickWidget* turn_joystick_ = nullptr;   /**< Dual Turn pad. */
  TeleopJoystickWidget* arcade_joystick_ = nullptr; /**< Arcade pad. */
  QLabel* hint_label_ = nullptr;        /**< Keyboard / smart-teleop hint. */
  QCheckBox* smart_teleop_check_ = nullptr; /**< Route via teleop goals. */
  bool key_forward_ = false;            /**< Keyboard: forward held. */
  bool key_back_ = false;               /**< Keyboard: back held. */
  bool key_left_ = false;               /**< Keyboard: left / turn left held. */
  bool key_right_ = false;              /**< Keyboard: right / turn right held. */
  bool keyboard_driving_ = false;       /**< @c true while keys drive sticks. */
  bool suppress_speed_signal_ = false;  /**< Guard while syncing speed spins. */
};

}  // namespace teleop
}  // namespace autoviz

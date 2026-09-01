/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

class TeleopControlWidget : public QWidget {
  Q_OBJECT

 public:
  explicit TeleopControlWidget(QWidget* parent = nullptr);

  void resetJoysticks();
  void setStickMode(TeleopStickMode mode);
  TeleopStickMode stickMode() const { return stick_mode_; }
  void setMaxSpeeds(double max_linear, double max_angular);
  double maxLinearSpeed() const { return max_linear_speed_; }
  double maxAngularSpeed() const { return max_angular_speed_; }
  void setSmartTeleopEnabled(bool enabled);
  bool smartTeleopEnabled() const { return smart_teleop_enabled_; }
  void setSmartTeleopAvailable(bool available);
  void setSmartTeleopStatusText(const QString& status);

 signals:
  /** Dual Move stick / Arcade stick Y: x = strafe (dual only), y = forward. */
  void linearChanged(double x, double y);
  /** Dual Turn stick X / Arcade stick X: turn. */
  void angularChanged(double turn);
  void linearReleased();
  void angularReleased();
  void stopClicked();
  void stickModeChanged(TeleopStickMode mode);
  void maxSpeedsChanged(double max_linear, double max_angular);
  void smartTeleopChanged(bool enabled);

 protected:
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;
  void focusInEvent(QFocusEvent* event) override;
  bool eventFilter(QObject* watched, QEvent* event) override;

 private:
  void applyModeUi();
  void emitArcadeFromStick(double x, double y, bool released);
  void updateArcadeFromKeyboard();
  void clearKeyboardState();
  void emitMaxSpeedsFromUi();
  void updateSmartTeleopHint();

  TeleopStickMode stick_mode_ = TeleopStickMode::kDual;
  bool smart_teleop_enabled_ = false;
  QString smart_teleop_status_text_;
  double max_linear_speed_ = 0.5;
  double max_angular_speed_ = 0.5;
  QButtonGroup* mode_group_ = nullptr;
  QAbstractButton* dual_mode_button_ = nullptr;
  QAbstractButton* arcade_mode_button_ = nullptr;
  QDoubleSpinBox* max_linear_spin_ = nullptr;
  QDoubleSpinBox* max_angular_spin_ = nullptr;
  QFrame* dual_frame_ = nullptr;
  QFrame* arcade_frame_ = nullptr;
  TeleopJoystickWidget* move_joystick_ = nullptr;
  TeleopJoystickWidget* turn_joystick_ = nullptr;
  TeleopJoystickWidget* arcade_joystick_ = nullptr;
  QLabel* hint_label_ = nullptr;
  QCheckBox* smart_teleop_check_ = nullptr;
  bool key_forward_ = false;
  bool key_back_ = false;
  bool key_left_ = false;
  bool key_right_ = false;
  bool keyboard_driving_ = false;
  bool suppress_speed_signal_ = false;
};

}  // namespace teleop
}  // namespace autoviz
